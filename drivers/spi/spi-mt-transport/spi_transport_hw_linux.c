// SPDX-License-Identifier: GPL-2.0
/*
 * spi_transport_hw_linux.c - Linux kernel HW-adapter for the MultiTracks SPI
 * transport core, Host role only. See spi_transport_hw_linux.h.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/completion.h>
#include <linux/sched.h>
#include <linux/hardirq.h>
#include <linux/atomic.h>

#include "spi_transport_hw_linux.h"
#include "spi_transport/spi_transport_frame.h"

/* MT-159369 bring-up instrumentation: cap the raw-buffer hex dump below to
 * the first few CRC failures -- enough to inspect the actual corruption
 * pattern by eye without flooding dmesg the way the per-event "link event:
 * ... CRC error" warnings already do. */
static atomic_t gCrcDumpRemaining = ATOMIC_INIT(8);

/* MT-159369 bring-up diagnostic: unconditional (not CRC-failure-gated, see
 * gCrcDumpRemaining above) raw dump of the first few TX/RX frames, for
 * direct cross-reference against the STM32 side's matching "fbs TX"/
 * "fbs RX" hex dumps (plat_fbs.cpp, commit a4a66d11) of the same early
 * exchange -- goal is localizing the dual-headerCrc non-determinism
 * finding to a specific hop. Two independent counters rather than a
 * paired per-transfer flag: unlike the STM32 side (which boots long
 * before Linux is up and has to gate on "first real armed transfer"),
 * this Host-side driver only starts arming once the tick thread is
 * already running against a live link, so the Nth TX dump and Nth RX
 * dump line up by simple chronological order in dmesg. */
static atomic_t gTxDumpRemaining = ATOMIC_INIT(5);
static atomic_t gRxDumpRemaining = ATOMIC_INIT(5);

/*
 * Bound for mt_hw_abort()'s wait on an in-flight transfer's completion.
 * spi_imx_dma_transfer()'s own internal timeout (spi_imx_calculate_timeout()
 * in drivers/spi/spi-imx.c) is unconditionally >= 2000ms (a flat "+1 second,
 * doubled" floor, regardless of this driver's small fixed frame size), and
 * spi_imx_transfer_one() calls it exactly once with no internal retry --
 * confirmed by reading both. 3000ms gives that floor comfortable scheduling
 * margin without the abort path itself becoming an unbounded stall.
 */
#define MT_HW_ABORT_TIMEOUT_MS 3000

/*
 * NSS/NRDY are driven/read as plain manually-owned GPIOs, not the SPI
 * subsystem's automatic per-message chip-select. The core holds NSS low
 * continuously across a whole request->ack->clock->complete cycle (which may
 * span more than one spi_async() call in the Client-initiated case), not
 * just the duration of one transfer -- the SPI core's built-in cs-gpios
 * handling only ever asserts CS for a single spi_message. Both GPIOs are
 * requested via a driver-private "multitracks,nss-gpios"/
 * "multitracks,nrdy-gpios" devicetree binding (see spi_mt_transport_drv.c)
 * rather than the standard "cs-gpios"
 * property, specifically so the SPI core never learns about them and never
 * tries to toggle them itself. Both are declared GPIO_ACTIVE_HIGH in the
 * devicetree regardless of the physical wire's true active sense, so that
 * gpiod_set_value_cansleep()'s logical value always equals the literal pin
 * level -- matching this whole contract's "true = pin HIGH" convention
 * exactly.
 *
 * All three accessors below use the _cansleep variants: none of these
 * calls happen from atomic/IRQ context -- mt_hw_select_assert() and
 * mt_hw_ready_read() run from the tick kthread, and
 * mt_transport_hw_linux_nrdy_irq() is registered as a threaded IRQ (NULL
 * primary handler), which by definition runs in a context where sleeping
 * is allowed. Using the plain (non-cansleep) accessors would be unsafe if
 * this GPIO ever ends up backed by a sleep-capable provider (e.g. an
 * I2C/SPI GPIO expander) instead of the native SoC GPIO controller this
 * board happens to use.
 */

static void mt_hw_spi_complete(void *context)
{
	struct mt_transport_hw_ctx *ctx = context;
	uint16_t length = ctx->msg.status == 0 ? ctx->xfer.len : 0;
	/* MT-159369 bring-up instrumentation -- see armedAt's struct comment. */
	s64 armToCompleteUs = ktime_us_delta(ktime_get(), ctx->armedAt);

	if (armToCompleteUs > 5000)
		dev_warn(&ctx->spi->dev,
			 "[MT-159369] slow transfer: arm-to-complete took %lldus (status=%d)\n",
			 armToCompleteUs, ctx->msg.status);
	else
		dev_dbg(&ctx->spi->dev, "[MT-159369] transfer complete: arm-to-complete %lldus\n",
			armToCompleteUs);

	/* MT-159369 bring-up instrumentation: dump the actual raw bytes the
	 * first few times a completed transfer fails header/payload CRC, so
	 * the real corruption pattern (single scattered bit-flips vs. a
	 * consistent byte-shift/offset vs. something else systematic) can be
	 * inspected directly instead of just counted. Checked here, before
	 * pOnTransferComplete() below hands the buffer to the core -- this is
	 * the same raw content the core's own CRC check will see. */
	if ((length > 0) && !spiTransportFrameHeaderCrcOk(ctx->xfer.rx_buf)) {
		if (atomic_dec_if_positive(&gCrcDumpRemaining) >= 0)
			print_hex_dump(KERN_ERR, "[MT-159369] hdrCrc-fail rx: ", DUMP_PREFIX_OFFSET,
				       16, 1, ctx->xfer.rx_buf, length, false);
	} else if ((length > 0) && !spiTransportFramePayloadCrcOk(ctx->xfer.rx_buf)) {
		if (atomic_dec_if_positive(&gCrcDumpRemaining) >= 0)
			print_hex_dump(KERN_ERR, "[MT-159369] payCrc-fail rx: ", DUMP_PREFIX_OFFSET,
				       16, 1, ctx->xfer.rx_buf, length, false);
	}

	/* MT-159369 bring-up diagnostic: unconditional raw RX dump for the
	 * first few completions -- see gRxDumpRemaining's doc comment. Not
	 * gated on CRC pass/fail (unlike the dumps just above), so this
	 * covers whatever the first few real exchanges actually look like,
	 * cross-referenceable against the STM32 side's "fbs RX" dump for the
	 * same window. */
	if ((length > 0) && (atomic_dec_if_positive(&gRxDumpRemaining) >= 0))
		print_hex_dump(KERN_ERR, "[MT-159369] RX raw: ", DUMP_PREFIX_OFFSET, 16, 1,
			       ctx->xfer.rx_buf, length, false);

	/* pOnTransferComplete() must run before transferComplete is signaled:
	 * mt_hw_abort() waits on this same completion, and it runs on the
	 * tick thread -- a different context than this SPI completion
	 * callback. Signaling first would let an aborting/woken tick thread
	 * proceed (and potentially call back into the core) while this
	 * context is still inside pOnTransferComplete() mutating core state,
	 * a concurrent unsynchronized access. Found by Copilot's PR #46
	 * review.
	 */
	if (ctx->pHw->pOnTransferComplete)
		ctx->pHw->pOnTransferComplete(ctx->pHw->pCoreCtx, length);

	/* Signal "msg/xfer no longer referenced by the SPI core, and the
	 * core has already been notified" only now -- pNotify may wake the
	 * tick thread straight into a new pTransferStart(), which gates on
	 * this same completion, so it's still correctly ordered after.
	 */
	complete(&ctx->transferComplete);

	if (ctx->pNotify)
		ctx->pNotify(ctx->pNotifyCtx);
}

static teSpiTransportError mt_hw_transfer_start(void *pContext, const uint8_t *pTx, uint8_t *pRx,
						 uint16_t length)
{
	struct mt_transport_hw_ctx *ctx = pContext;
	int ret;

	/* msg/xfer are shared across every transfer (see the struct comment)
	 * -- reinitializing them while the SPI core still has the previous
	 * submission queued/in-flight corrupts its internal message-queue and
	 * scatterlist state. mt_hw_abort() is supposed to guarantee this is
	 * clear before the core ever calls back in here again, so hitting
	 * this is itself a bug elsewhere; refuse rather than corrupt state.
	 */
	if (!completion_done(&ctx->transferComplete)) {
		/* MT-159369 bring-up instrumentation: how long has the stale
		 * transfer already been outstanding at the moment this new
		 * arm is refused? See armedAt's struct comment. */
		s64 outstandingUs = ktime_us_delta(ktime_get(), ctx->armedAt);

		dev_err(&ctx->spi->dev,
			"pTransferStart() called with a previous transfer still in flight (outstanding %lldus, caller=%s in_irq=%d in_softirq=%d) -- refusing to reinitialize shared msg/xfer state\n",
			outstandingUs, current->comm, (int)in_irq(), (int)in_softirq());
		return eSpiTransportErrorHardwareFailure;
	}
	reinit_completion(&ctx->transferComplete);

	/* MT-159369 bring-up diagnostic: unconditional raw TX dump for the
	 * first few arms -- see gTxDumpRemaining's doc comment above. This is
	 * "what we're about to hand to spi_async()", captured before anything
	 * else touches pTx/pRx this call, for direct cross-reference against
	 * the STM32 side's "fbs TX" dump (what it believes it armed) for the
	 * same early exchange. */
	if (atomic_dec_if_positive(&gTxDumpRemaining) >= 0)
		print_hex_dump(KERN_ERR, "[MT-159369] TX raw: ", DUMP_PREFIX_OFFSET, 16, 1, pTx,
			       length, false);

	/* MT-159369 bring-up instrumentation: poison the RX buffer with a
	 * sentinel pattern before every arm, distinct from any real frame
	 * byte value the protocol would ever legitimately send (0xA5/0x5A
	 * magic, mostly-zero payloads, small CRC/seq values). rx_buf is a
	 * single, fixed buffer reused across every transfer (see the struct
	 * comment on msg/xfer) -- if a completed ("successful") transfer's
	 * dump still shows this sentinel anywhere, that byte was never
	 * actually written by DMA, proving a short/partial transfer rather
	 * than a fully-fresh 128 bytes. Deliberately poisoning the buffer
	 * the core is about to hand to hardware, not just reading stale
	 * content after the fact -- rules out "it was already zero from a
	 * previous frame" as an alternate explanation for an all-zero
	 * payload region. */
	memset(pRx, 0x37, length);

	spi_message_init(&ctx->msg);
	memset(&ctx->xfer, 0, sizeof(ctx->xfer));
	ctx->xfer.tx_buf = pTx;
	ctx->xfer.rx_buf = pRx;
	ctx->xfer.len = length;
	spi_message_add_tail(&ctx->xfer, &ctx->msg);
	ctx->msg.complete = mt_hw_spi_complete;
	ctx->msg.context = ctx;

	ctx->armedAt = ktime_get(); /* MT-159369 bring-up instrumentation */

	ret = spi_async(ctx->spi, &ctx->msg);
	if (ret) {
		dev_dbg(&ctx->spi->dev, "spi_async failed: %d\n", ret);
		/* No async completion will ever fire for this failed
		 * submission -- release the in-flight guard ourselves.
		 */
		complete(&ctx->transferComplete);
		return eSpiTransportErrorHardwareFailure;
	}

	return eSpiTransportErrorNone;
}

/* Host only: drive NSS. No-op on Client, but this adapter only ever runs
 * Host role, so unconditionally drive the line.
 */
static void mt_hw_select_assert(void *pContext, bool high)
{
	struct mt_transport_hw_ctx *ctx = pContext;

	gpiod_set_value_cansleep(ctx->nss_gpiod, high ? 1 : 0);
}

/* Client only -- Host never calls this; left wired to a harmless stub so a
 * stray call (there should never be one) doesn't crash rather than silently
 * doing nothing unexpected.
 */
static void mt_hw_ready_assert(void *pContext, bool high)
{
	struct mt_transport_hw_ctx *ctx = pContext;

	(void)high;
	dev_warn_once(&ctx->spi->dev, "pReadyAssert called on Host role adapter (unexpected)\n");
}

static bool mt_hw_ready_read(void *pContext)
{
	struct mt_transport_hw_ctx *ctx = pContext;
	int val = gpiod_get_value_cansleep(ctx->nrdy_gpiod);

	/* A negative errno (GPIO provider failure) must not fall through the
	 * old bare ternary, which mapped any nonzero result -- errno included
	 * -- to true. Fail closed instead: report not-ready rather than
	 * risk clocking the peer on an invalid handshake. Found by Copilot's
	 * PR #46 review.
	 */
	if (val < 0) {
		dev_err_ratelimited(&ctx->spi->dev, "NRDY GPIO read failed: %d\n", val);
		return false;
	}

	return val ? true : false;
}

/*
 * Force the transfer engine back to idle after a wedged pTransferStart().
 * Unlike STM32 HAL (which needs a manual RCC-level peripheral reset), the
 * i.MX8MM's spi-imx controller driver already runs its own
 * completion-timeout + dmaengine_terminate_all() + reset recovery internally
 * on a stuck DMA transfer (drivers/spi/spi-imx.c transfer_one()). The Linux
 * SPI core also has no public master-mode equivalent of HAL_SPI_Abort() --
 * spi_slave_abort() is slave-mode only.
 *
 * Confirmed live on the EVK (MT-158682): spi-imx's internal recovery is NOT
 * sufficient on its own, because it isn't synchronous with this call. The
 * core's own disconnect watchdog (SPI_TRANSPORT_DISCONNECT_MS, 1500ms) fires
 * before spi_imx_calculate_timeout()'s unconditional >=2000ms floor can
 * possibly have elapsed, so a log-only pAbort() let the retry that follows
 * reinitialize msg/xfer (see mt_hw_transfer_start()) while spi_imx was still
 * blocked inside its own wait_for_completion_timeout() referencing that same
 * memory -- corrupting the SPI core's message queue/scatterlist state and
 * crashing with a NULL deref in spi_imx_dma_transfer()'s sg_last(). This
 * contract has no return value (must be safe to call whether or not
 * anything is armed, and the core proceeds regardless of what happens here),
 * so the only correct fix available is to actually block until spi-imx's own
 * bounded recovery has had time to finish before returning.
 */
static void mt_hw_abort(void *pContext)
{
	struct mt_transport_hw_ctx *ctx = pContext;

	if (completion_done(&ctx->transferComplete))
		return;

	if (!wait_for_completion_timeout(&ctx->transferComplete,
					  msecs_to_jiffies(MT_HW_ABORT_TIMEOUT_MS))) {
		dev_err(&ctx->spi->dev,
			"pAbort(): transfer still in flight %ums after spi-imx's own DMA-timeout recovery should have finished -- proceeding anyway, next transfer may still race\n",
			MT_HW_ABORT_TIMEOUT_MS);
	}

	/* Restore the "idle, no transfer in flight" resting state for the
	 * next mt_hw_transfer_start(), whether we got here via a genuine
	 * completion or the timeout fallback above -- wait_for_completion_*
	 * consumes the completion on success, and the timeout path never
	 * signaled it in the first place.
	 */
	complete(&ctx->transferComplete);
}

void mt_transport_hw_linux_init(struct mt_transport_hw_ctx *ctx, struct spi_device *spi,
				 struct gpio_desc *nss_gpiod, struct gpio_desc *nrdy_gpiod,
				 trSpiTransportHw *pHw)
{
	memset(ctx, 0, sizeof(*ctx));
	ctx->spi = spi;
	ctx->nss_gpiod = nss_gpiod;
	ctx->nrdy_gpiod = nrdy_gpiod;

	/* Starts "done" -- idle, no transfer in flight yet. */
	init_completion(&ctx->transferComplete);
	complete(&ctx->transferComplete);

	ctx->pHw = pHw;

	memset(pHw, 0, sizeof(*pHw));
	pHw->pTransferStart = mt_hw_transfer_start;
	pHw->pSelectAssert = mt_hw_select_assert;
	pHw->pReadyAssert = mt_hw_ready_assert;
	pHw->pReadyRead = mt_hw_ready_read;
	pHw->pAbort = mt_hw_abort;
	pHw->pContext = ctx;

	/* pOnSelectEvent/pOnClockStart deliberately left NULL -- Client-only
	 * concepts the core's Host code paths never invoke. pOnReadyEvent is
	 * wired later if an NRDY IRQ is available (see
	 * mt_transport_hw_linux_nrdy_irq()); polling pReadyRead() via the tick
	 * loop always works as the fallback.
	 */
}

void mt_transport_hw_linux_set_notify(struct mt_transport_hw_ctx *ctx,
				      void (*pNotify)(void *pNotifyCtx), void *pNotifyCtx)
{
	ctx->pNotify = pNotify;
	ctx->pNotifyCtx = pNotifyCtx;
}

irqreturn_t mt_transport_hw_linux_nrdy_irq(int irq, void *dev_id)
{
	struct mt_transport_hw_ctx *ctx = dev_id;
	int val = gpiod_get_value_cansleep(ctx->nrdy_gpiod);

	/* Same errno-to-true bug as mt_hw_ready_read(), but here a
	 * misreported level would advance the Host state machine on a bad
	 * handshake -- skip the event entirely on error instead of guessing
	 * a level; the tick thread's own pReadyRead() polling remains
	 * available as a fallback. Found by Copilot's PR #46 review.
	 */
	if (val < 0) {
		dev_err_ratelimited(&ctx->spi->dev, "NRDY GPIO read failed in IRQ handler: %d\n",
				    val);
		return IRQ_HANDLED;
	}

	if (ctx->pHw->pOnReadyEvent)
		ctx->pHw->pOnReadyEvent(ctx->pHw->pCoreCtx, val ? true : false);

	if (ctx->pNotify)
		ctx->pNotify(ctx->pNotifyCtx);

	return IRQ_HANDLED;
}
