// SPDX-License-Identifier: GPL-2.0
/*
 * spi_mt_transport_drv.c - Linux kernel Host-role driver for the MultiTracks
 * SPI transport protocol. Builds in-tree (see drivers/spi/Kconfig's
 * CONFIG_SPI_MT_TRANSPORT and this directory's Makefile) against the
 * portable protocol core, which is NOT committed in this repo -- it's the
 * firmware repo's firmware-common/spi-transport/{src,inc}/ single source
 * of truth, staged into this directory's gitignored core/ subdirectory at
 * Yocto build time (see imx8mmini-bb-evk's meta-mt-transport-evk
 * linux-imx_%.bbappend's do_patch postfunc).
 *
 * Talks Host role to an STM32-class Client peer over a raw SPI bus plus a
 * companion NRDY GPIO handshake line. See the firmware repo's
 * firmware-common/spi-transport/docs/ProtocolSpec.md for the wire protocol
 * and handshake state machine this ports into the kernel.
 *
 * Scope note (MT-158113): this is the driver only. The EVK-side test
 * framework (MT-158682) is a separate ticket -- the userspace interface
 * below (one misc device per raw channel + a small per-channel TX ring, see
 * mt_transport_tx_service()) is still a placeholder ahead of a real config
 * surface: single in-flight RX message per channel, no ioctl/config
 * surface. Channel *count* is generic, though (MT-159369): every raw
 * channel the core supports (SPI_TRANSPORT_CHANNELS_MAX - 1, currently 19)
 * gets a misc device, registered unconditionally at probe() time, whether
 * or not anything uses it yet. What a given channel number actually
 * carries (Transport Services on 1, Property Model on 2, the legacy IPC
 * tunnel on 3, future MIDI/debug/vendor channels on 4+) is a
 * userspace-level fact only (firmware-common/mt_ipc_fbs/docs/ChannelMapping.md)
 * -- this driver has no notion of channel purpose, and adding a new one
 * never requires a kernel change here again.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/atomic.h>
#include <linux/kref.h>

#include "spi_transport/spi_transport.h"
#include "spi_transport/spi_transport_types.h"
#include "spi_transport_os_linux.h"
#include "spi_transport_hw_linux.h"

#define DRIVER_NAME "spi-mt-transport"
#define MT_TRANSPORT_TX_QUEUE_DEPTH 10

/* Every raw channel the spi-transport core supports
 * (SPI_TRANSPORT_CHANNELS_MAX) except channel 0 (the transport's own
 * control/handshake channel, never userspace-visible) gets a misc device --
 * see this file's header. Channel 1 keeps the original single-channel
 * driver's unprefixed misc device name ("mt_spi_transport") for backward
 * compatibility with existing tools/scripts; every other channel is
 * "mt_spi_transport_ch<N>" (see MT_TRANSPORT_MISC_NAME_MAX).
 */
#define MT_TRANSPORT_NUM_CHANNELS (SPI_TRANSPORT_CHANNELS_MAX - 1u)
/* "mt_spi_transport_ch" (20) + up to 3 digits + NUL fits SPI_TRANSPORT_CHANNELS_MAX
 * values up to 999 -- reassess if that constant ever grows past 3 digits.
 */
#define MT_TRANSPORT_MISC_NAME_MAX 24

struct mt_transport_tx_slot {
	uint8_t buf[SPI_TRANSPORT_CHANNEL_MESSAGE_MAX];
	uint16_t len;
};

struct mt_transport_priv;

/* Everything that used to be single-instance state directly in struct
 * mt_transport_priv but is actually per-channel -- split out here so a
 * second channel (Property Model, MT-148208) can be added without
 * duplicating the whole driver. See struct mt_transport_priv's own comment
 * for what stays link-level instead of moving here.
 */
struct mt_transport_channel {
	struct mt_transport_priv *priv; /* back-pointer -- needed for the
					  * shared removed/refcount/htransport
					  * checks in the open/release/read/
					  * write/poll functions below, which
					  * used to reach priv directly via
					  * container_of() on a single shared
					  * misc device.
					  */
	uint8_t channel_num;
	char misc_name[MT_TRANSPORT_MISC_NAME_MAX]; /* backing storage for
						       * misc.name -- must
						       * outlive
						       * misc_register(), so
						       * can't be a probe()-
						       * local buffer.
						       */

	/* Minimal placeholder userspace interface -- MT-158682 owns the real
	 * design. Single hardcoded channel per device, single in-flight RX
	 * message, blocking read()/write(), best-effort poll().
	 */
	struct miscdevice misc;
	wait_queue_head_t rx_wq;
	spinlock_t rx_lock;
	uint8_t rx_buf[SPI_TRANSPORT_CHANNEL_MESSAGE_MAX];
	uint16_t rx_len;
	bool rx_valid;

	/* TX ring (see mt_transport_misc_write()/mt_transport_tx_service()).
	 * spiTransportSend() borrows pBuffer -- per
	 * spi_transport_channel.h's trSpiTransportChannelSlot comment, the
	 * core keeps using it until the whole message finishes sending,
	 * which can span multiple ticks -- so each slot needs
	 * driver-instance lifetime, not a write()-local stack array (the
	 * latter is a use-after-return once the syscall returns and its
	 * frame is torn down).
	 *
	 * A slot is only ever reused once a *later* spiTransportSend() call
	 * succeeds: the core enforces a single in-flight message per channel
	 * (txPending only clears when the previous message's last chunk is
	 * confirmed sent), so that later success is itself proof the
	 * previous slot is done -- not a timing guess. This lets write()
	 * enqueue and return immediately instead of blocking on a flat
	 * drain wait; tx_lock protects only the ring's head/tail/count
	 * bookkeeping (no sleeping calls under it).
	 */
	spinlock_t tx_lock;
	struct mt_transport_tx_slot tx_slots[MT_TRANSPORT_TX_QUEUE_DEPTH];
	unsigned int tx_head; /* next slot index to submit */
	unsigned int tx_tail; /* next free slot index to fill */
	/* Slots written by write() but not yet handed to spiTransportSend() --
	 * deliberately NOT counting the in-flight slot too (see
	 * mt_transport_tx_service()'s comment on why conflating the two was a
	 * real bug). The room check callers need is
	 * tx_queued_count + (tx_in_flight_idx >= 0 ? 1 : 0) < DEPTH.
	 */
	unsigned int tx_queued_count;
	int tx_in_flight_idx; /* Set on every successful spiTransportSend(),
				* but never cleared back to -1 -- the core's
				* public API has no TX-completion signal to
				* clear it on (tracked as MT-158925 item 1),
				* so this permanently costs one slot of room
				* in mt_transport_tx_room_locked() from the
				* first successful send onward. Not a
				* correctness bug (no data loss, no crash --
				* just DEPTH-1 usable slots instead of DEPTH),
				* and not fixable here without the core API
				* change MT-158925 requests; a driver-side
				* guess (e.g. a timeout) would reintroduce
				* the exact anti-pattern that ticket already
				* flags as having cost ~200x throughput
				* previously. Found by Copilot's PR #46
				* review.
				*/
	wait_queue_head_t tx_free_wq;

	atomic_t available; /* 1 = no fd currently open on this channel's
			      * device, 0 = one is -- enforces single-open
			      * semantics per channel, see
			      * mt_transport_misc_open(). This also closes
			      * the concurrent-reader race Copilot's review
			      * separately flagged in
			      * mt_transport_misc_read(): with at most one
			      * open fd per channel, there is only ever one
			      * reader on that channel.
			      */
};

struct mt_transport_priv {
	struct spi_device *spi;
	struct device *dev;

	struct gpio_desc *nss_gpiod;
	struct gpio_desc *nrdy_gpiod;
	int nrdy_irq;
	bool nrdy_irq_requested; /* only true once devm_request_threaded_irq()
				   * actually succeeded -- see mt_transport_remove().
				   */

	trSpiTransportOs os;
	trSpiTransportHw hw;
	struct mt_transport_os_ctx os_ctx;
	struct mt_transport_hw_ctx hw_ctx;
	thSpiTransport htransport;

	struct task_struct *tick_thread;

	/* One entry per raw channel (index 0 = channel 1, the original
	 * single-channel device's unchanged name/number, through index
	 * MT_TRANSPORT_NUM_CHANNELS - 1 = channel SPI_TRANSPORT_CHANNELS_MAX - 1)
	 * -- see MT_TRANSPORT_NUM_CHANNELS's comment and struct
	 * mt_transport_channel.
	 */
	struct mt_transport_channel channels[MT_TRANSPORT_NUM_CHANNELS];

	/* Link-wide event counters -- mirrors the STM32 Client harness's
	 * [DBG] conn=/disc=/hdrCrc=/payCrc=/seq=/dmaFail=/dmaTo= naming
	 * (firmware-common/spi-transport/test/stm32-disco/app/, in the
	 * firmware repo) so a fault-injection run's peer-side verdict can
	 * actually be read off this Host, not just inferred from the absence
	 * of a crash. Before this, mt_transport_event_callback() only logged
	 * via dev_dbg(), invisible in dmesg without dynamic debug explicitly
	 * enabled -- confirmed live: zero log output across ~50 real
	 * connect/disconnect cycles and dozens of DMA-failure injections
	 * during hardware bring-up (MT-158113). atomic_t: incremented from
	 * the tick thread (mt_transport_event_callback(), single-threaded),
	 * read from arbitrary userspace context via sysfs. Link-level, not
	 * per-channel -- connect/disconnect/CRC/DMA events happen to the
	 * physical link, not to one channel of it.
	 */
	atomic_t evt_connected;
	atomic_t evt_disconnected;
	atomic_t evt_hdr_crc;
	atomic_t evt_payload_crc;
	atomic_t evt_seq_gap;
	atomic_t evt_dma_failure;
	atomic_t evt_dma_timeout;

	/* priv is kzalloc'd, not devm_kzalloc'd (see mt_transport_probe()):
	 * the embedded miscdevices can outlive the SPI device across an
	 * unbind if userspace still holds a channel's device open, so
	 * something other than devm has to own freeing this memory --
	 * kref does, with one reference held by the driver instance itself
	 * (dropped in mt_transport_remove()) and one more per open fd, on
	 * either channel (mt_transport_misc_open()/_release()). Found by
	 * Copilot's PR #46 review.
	 */
	struct kref refcount;
	bool removed; /* true once mt_transport_remove() has torn down the
		       * transport core/tick thread -- read()/write() must
		       * bail out with -ENODEV rather than touching
		       * now-invalid os/hw state via an fd that outlived
		       * unbind.
		       */
};

/* Wakes the tick kthread -- shared by the SPI-completion path and the
 * (optional) NRDY-IRQ path, both of which only ever need to say "something
 * happened, re-run spiTransportTick() soon" rather than touch core state
 * directly from interrupt context.
 */
static void mt_transport_tick_notify(void *pNotifyCtx)
{
	struct mt_transport_priv *priv = pNotifyCtx;

	priv->os.pTaskNotifyGive(priv->os.pContext);
}

static void mt_transport_rx_callback(void *pContext, uint8_t channel, const uint8_t *pBuffer,
				      uint16_t length, uint8_t flags)
{
	struct mt_transport_channel *chan = pContext;
	unsigned long irqflags;

	(void)flags;
	/* pContext already identifies which channel this is -- each channel
	 * is registered with its own struct mt_transport_channel* as
	 * context (see mt_transport_probe()), so no channel-number branch
	 * is needed here.
	 */
	(void)channel;
	if (length > sizeof(chan->rx_buf))
		length = sizeof(chan->rx_buf);

	spin_lock_irqsave(&chan->rx_lock, irqflags);
	memcpy(chan->rx_buf, pBuffer, length);
	chan->rx_len = length;
	/* WRITE_ONCE() pairs with the unlocked READ_ONCE() reads of rx_valid
	 * in mt_transport_misc_poll() and the wait_event_interruptible()
	 * condition below -- rx_buf/rx_len are only ever touched under
	 * rx_lock, but rx_valid itself is also read lock-free in those two
	 * spots (both are the standard Linux poll_wait()/wait_event idiom,
	 * where the wait/wake primitives themselves provide the needed
	 * ordering -- this is about being explicit for readers/tooling like
	 * KCSAN, not fixing an actual race).
	 */
	WRITE_ONCE(chan->rx_valid, true);
	spin_unlock_irqrestore(&chan->rx_lock, irqflags);

	wake_up_interruptible(&chan->rx_wq);
}

static void mt_transport_event_callback(void *pContext, teSpiTransportEvent eEvent)
{
	/* pContext is the same struct mt_transport_channel* passed to
	 * spiTransportRegisterChannel() for both callbacks on this channel
	 * (see mt_transport_probe()) -- events are link-level, not
	 * per-channel, so derive priv from chan rather than assuming
	 * pContext already IS priv. Passing chan straight through as
	 * priv here (both are the first-member-is-a-pointer shape) used to
	 * silently reinterpret chan's own fields as priv's, corrupting
	 * priv->dev into a garbage pointer -- confirmed on hardware as a
	 * NULL-deref Oops in __dev_printk() the first time this callback
	 * fired.
	 */
	struct mt_transport_channel *chan = pContext;
	struct mt_transport_priv *priv    = chan->priv;

	/* MT-159369: these were plain dev_info()/dev_warn() -- unlike
	 * spi_transport_hw_linux.c's dev_err_ratelimited() for the analogous
	 * NRDY-read-failure case, nothing here throttled the print rate.
	 * Fine for the bounded, human-supervised bring-up testing this was
	 * written for (MT-158113: ~50 connect/disconnect cycles, a few dozen
	 * injected DMA failures), but a real link-quality storm can raise
	 * these into the hundreds/sec. imx_uart_console_write() holds
	 * port.lock with local IRQs (and RRDYEN, its own RX-ready IRQ)
	 * disabled for the full synchronous, poll-driven duration of each
	 * line at the console baud rate -- confirmed live: this is why
	 * serial BREAK + magic sysrq got no response during the hang, not a
	 * genuine deadlock. Back-to-back events leave no gap for anything
	 * else -- including the RX path a BREAK needs, and the timer tick
	 * the softlockup/hung-task watchdogs themselves need -- to run.
	 * _ratelimited keeps these visible without the storm.
	 */
	switch (eEvent) {
	case eSpiTransportEventConnected:
		atomic_inc(&priv->evt_connected);
		dev_info_ratelimited(priv->dev, "link event: connected\n");
		break;
	case eSpiTransportEventDisconnected:
		atomic_inc(&priv->evt_disconnected);
		dev_info_ratelimited(priv->dev, "link event: disconnected\n");
		break;
	case eSpiTransportEventErrorHeaderCrc:
		atomic_inc(&priv->evt_hdr_crc);
		dev_warn_ratelimited(priv->dev, "link event: header CRC error\n");
		break;
	case eSpiTransportEventErrorPayloadCrc:
		atomic_inc(&priv->evt_payload_crc);
		dev_warn_ratelimited(priv->dev, "link event: payload CRC error\n");
		break;
	case eSpiTransportEventErrorSequenceGap:
		atomic_inc(&priv->evt_seq_gap);
		dev_warn_ratelimited(priv->dev, "link event: sequence gap\n");
		break;
	case eSpiTransportEventErrorDmaFailure:
		atomic_inc(&priv->evt_dma_failure);
		dev_warn_ratelimited(priv->dev, "link event: DMA arm failure\n");
		break;
	case eSpiTransportEventErrorDmaTimeout:
		atomic_inc(&priv->evt_dma_timeout);
		dev_warn_ratelimited(priv->dev, "link event: DMA timeout\n");
		break;
	default:
		dev_warn_ratelimited(priv->dev, "link event: unknown (%d)\n", (int)eEvent);
		break;
	}
}

/*
 * Submit the oldest queued TX slot (if any) via spiTransportSend(). A
 * success return proves the *previous* in-flight slot (if any) is now
 * done -- the core only accepts a new send once the last one's final
 * chunk is confirmed -- so that previous slot is freed right here, not
 * after a guessed timeout. Called once per channel per tick thread
 * iteration; a Busy return just means retry next tick, no state changes.
 *
 * Gates on tx_queued_count, not "is anything occupied at all": an earlier
 * version checked the combined queued+in-flight total, which let tx_head
 * advance onto a slot write() had never actually filled whenever exactly
 * one message was in flight and nothing new had been queued behind it --
 * tx_service() would then resend whatever stale bytes happened to be
 * sitting in that slot (found via Copilot PR review). Tracking queued
 * count separately from "is one slot in flight" makes "is there anything
 * NEW to submit" the only thing this check needs to answer.
 */
static void mt_transport_tx_service(struct mt_transport_channel *chan)
{
	unsigned long flags;
	unsigned int idx;
	uint16_t len;
	teSpiTransportError err;

	spin_lock_irqsave(&chan->tx_lock, flags);
	if (chan->tx_queued_count == 0) {
		spin_unlock_irqrestore(&chan->tx_lock, flags);
		return;
	}
	idx = chan->tx_head;
	len = chan->tx_slots[idx].len;
	spin_unlock_irqrestore(&chan->tx_lock, flags);

	err = spiTransportSend(chan->priv->htransport, chan->channel_num, chan->tx_slots[idx].buf,
				len, true);
	if (err != eSpiTransportErrorNone)
		return;

	spin_lock_irqsave(&chan->tx_lock, flags);
	chan->tx_queued_count--;
	chan->tx_in_flight_idx = idx;
	chan->tx_head = (chan->tx_head + 1) % MT_TRANSPORT_TX_QUEUE_DEPTH;
	spin_unlock_irqrestore(&chan->tx_lock, flags);
	wake_up_interruptible(&chan->tx_free_wq);
}

static int mt_transport_tick_thread_fn(void *data)
{
	struct mt_transport_priv *priv = data;
	int i;

	while (!kthread_should_stop()) {
		priv->os.pTaskNotifyWait(priv->os.pContext, 2);
		spiTransportTick(priv->htransport);
		for (i = 0; i < MT_TRANSPORT_NUM_CHANNELS; i++)
			mt_transport_tx_service(&priv->channels[i]);
	}
	return 0;
}

/* --- Minimal userspace interface (placeholder ahead of MT-158682) --- */

static void mt_transport_priv_release(struct kref *kref)
{
	struct mt_transport_priv *priv = container_of(kref, struct mt_transport_priv, refcount);

	kfree(priv);
}

static int mt_transport_misc_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *misc = filp->private_data;
	struct mt_transport_channel *chan = container_of(misc, struct mt_transport_channel, misc);
	struct mt_transport_priv *priv = chan->priv;

	(void)inode;

	if (READ_ONCE(priv->removed))
		return -ENODEV;

	if (!atomic_dec_and_test(&chan->available)) {
		atomic_inc(&chan->available);
		return -EBUSY;
	}

	kref_get(&priv->refcount);
	return 0;
}

static int mt_transport_misc_release(struct inode *inode, struct file *filp)
{
	struct miscdevice *misc = filp->private_data;
	struct mt_transport_channel *chan = container_of(misc, struct mt_transport_channel, misc);
	struct mt_transport_priv *priv = chan->priv;

	(void)inode;

	atomic_inc(&chan->available);
	kref_put(&priv->refcount, mt_transport_priv_release);
	return 0;
}

static ssize_t mt_transport_misc_read(struct file *filp, char __user *buf, size_t count,
				       loff_t *ppos)
{
	struct miscdevice *misc = filp->private_data;
	struct mt_transport_channel *chan = container_of(misc, struct mt_transport_channel, misc);
	struct mt_transport_priv *priv = chan->priv;
	uint8_t scratch[SPI_TRANSPORT_CHANNEL_MESSAGE_MAX];
	unsigned long irqflags;
	uint16_t len;
	int ret;

	(void)ppos;

	/* POSIX: a count of 0 must return 0 with no other effect -- must not
	 * block, and must not consume a pending message.
	 */
	if (count == 0)
		return 0;

	if (READ_ONCE(priv->removed))
		return -ENODEV;

	/* The single-open check in mt_transport_misc_open() only prevents a
	 * second fd -- it does nothing to stop two threads issuing read() on
	 * the *same* fd concurrently, which POSIX permits. Both could
	 * observe rx_valid true (via the O_NONBLOCK check below, or both
	 * waking from the same wait_event_interruptible()) and race into the
	 * consume block; without rechecking under the lock immediately before
	 * consuming, the loser would copy the winner's already-cleared
	 * rx_buf/rx_len as if it were a fresh message. Looping back to
	 * wait/retry instead of falling through fixes this: only the thread
	 * that actually observes rx_valid true *under rx_lock* consumes it,
	 * and rx_valid is cleared in that same critical section. Found by
	 * Copilot's PR #46 review.
	 */
	for (;;) {
		spin_lock_irqsave(&chan->rx_lock, irqflags);
		if (chan->rx_valid) {
			/* Snapshot into a local buffer under the lock, then
			 * copy_to_user() outside it -- copy_to_user() can
			 * fault/sleep, which is illegal while holding a
			 * spinlock. rx_valid is cleared here too (not after
			 * the copy) since it's rx_lock-protected state, same
			 * as rx_buf -- a failing copy_to_user (a broken
			 * caller's bad pointer) now consumes the buffered
			 * message rather than leaving it for retry, a minor,
			 * acceptable behavior change for this placeholder
			 * interface.
			 */
			len = chan->rx_len;
			if (len > count)
				len = count;
			memcpy(scratch, chan->rx_buf, len);
			WRITE_ONCE(chan->rx_valid, false);
			spin_unlock_irqrestore(&chan->rx_lock, irqflags);
			break;
		}
		spin_unlock_irqrestore(&chan->rx_lock, irqflags);

		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		/* removed is included in the wait condition (not just checked
		 * up front) so a reader already blocked here when
		 * mt_transport_remove() runs actually wakes up instead of
		 * sleeping forever -- remove() wakes rx_wq right after
		 * setting removed for exactly this reason. Found by
		 * Copilot's PR #46 review.
		 */
		ret = wait_event_interruptible(chan->rx_wq,
						READ_ONCE(chan->rx_valid) || READ_ONCE(priv->removed));
		if (ret)
			return ret;
		if (READ_ONCE(priv->removed))
			return -ENODEV;
	}

	if (copy_to_user(buf, scratch, len))
		return -EFAULT;

	return len;
}

/*
 * True if a new slot can be enqueued. Caller must already hold tx_lock --
 * occupied total is tx_queued_count (not-yet-submitted slots) plus one
 * more if a slot is currently in flight (tx_in_flight_idx >= 0), since
 * that slot is still reserved even though it doesn't count toward
 * tx_queued_count.
 */
static inline bool mt_transport_tx_room_locked(struct mt_transport_channel *chan)
{
	unsigned int occupied = chan->tx_queued_count + (chan->tx_in_flight_idx >= 0 ? 1 : 0);

	return occupied < MT_TRANSPORT_TX_QUEUE_DEPTH;
}

/*
 * wait_event_interruptible()'s condition check only -- takes and releases
 * tx_lock itself since it must be callable without already holding it.
 * mt_transport_misc_write()'s own room check below calls
 * mt_transport_tx_room_locked() directly instead (already holding the
 * lock at that point) rather than this wrapper, and deliberately so: that
 * check has to stay under the *same* lock acquisition that immediately
 * follows (the enqueue), otherwise a second writer could take the
 * now-free slot in the gap between checking and re-locking.
 */
static bool mt_transport_tx_has_room(struct mt_transport_channel *chan)
{
	unsigned long flags;
	bool room;

	spin_lock_irqsave(&chan->tx_lock, flags);
	room = mt_transport_tx_room_locked(chan);
	spin_unlock_irqrestore(&chan->tx_lock, flags);
	return room;
}

static ssize_t mt_transport_misc_write(struct file *filp, const char __user *buf, size_t count,
					loff_t *ppos)
{
	struct miscdevice *misc = filp->private_data;
	struct mt_transport_channel *chan = container_of(misc, struct mt_transport_channel, misc);
	struct mt_transport_priv *priv = chan->priv;
	uint8_t scratch[SPI_TRANSPORT_CHANNEL_MESSAGE_MAX];
	unsigned long flags;
	unsigned int idx;
	size_t len = count;
	int ret;

	(void)ppos;

	/* POSIX: a count of 0 must return 0 with no other effect -- must not
	 * enqueue a 0-length transport message.
	 */
	if (count == 0)
		return 0;

	if (READ_ONCE(priv->removed))
		return -ENODEV;

	if (len > sizeof(scratch))
		len = sizeof(scratch);
	if (copy_from_user(scratch, buf, len))
		return -EFAULT;

	for (;;) {
		spin_lock_irqsave(&chan->tx_lock, flags);
		if (mt_transport_tx_room_locked(chan))
			break;
		spin_unlock_irqrestore(&chan->tx_lock, flags);

		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		/* removed is included in the wait condition for the same
		 * reason as mt_transport_misc_read()'s rx_wq wait -- a
		 * writer already blocked here on unbind must not sleep
		 * forever; remove() wakes tx_free_wq right after setting
		 * removed. Found by Copilot's PR #46 review.
		 */
		ret = wait_event_interruptible(chan->tx_free_wq,
						mt_transport_tx_has_room(chan) || READ_ONCE(priv->removed));
		if (ret)
			return ret;
		if (READ_ONCE(priv->removed))
			return -ENODEV;
	}

	idx = chan->tx_tail;
	memcpy(chan->tx_slots[idx].buf, scratch, len);
	chan->tx_slots[idx].len = len;
	chan->tx_tail = (chan->tx_tail + 1) % MT_TRANSPORT_TX_QUEUE_DEPTH;
	chan->tx_queued_count++;
	spin_unlock_irqrestore(&chan->tx_lock, flags);

	/* Kick the tick thread so mt_transport_tx_service() attempts this
	 * send right away instead of waiting up to its ~2ms poll interval.
	 */
	priv->os.pTaskNotifyGive(priv->os.pContext);

	return len;
}

static __poll_t mt_transport_misc_poll(struct file *filp, poll_table *wait)
{
	struct miscdevice *misc = filp->private_data;
	struct mt_transport_channel *chan = container_of(misc, struct mt_transport_channel, misc);
	struct mt_transport_priv *priv = chan->priv;
	__poll_t mask = 0;

	poll_wait(filp, &chan->rx_wq, wait);
	/* Reported so an fd that outlives unbind can detect teardown via
	 * poll() instead of only finding out on its next read()/write() --
	 * rx_wq is woken on removal (see mt_transport_remove()), so this
	 * check is reachable rather than only ever seen on a fresh poll().
	 * Found by Copilot's PR #46 review.
	 */
	if (READ_ONCE(priv->removed))
		return EPOLLHUP | EPOLLERR;
	if (READ_ONCE(chan->rx_valid))
		mask |= EPOLLIN | EPOLLRDNORM;
	return mask;
}

static const struct file_operations mt_transport_misc_fops = {
	.owner = THIS_MODULE,
	.open = mt_transport_misc_open,
	.release = mt_transport_misc_release,
	.read = mt_transport_misc_read,
	.write = mt_transport_misc_write,
	.poll = mt_transport_misc_poll,
	/* This device doesn't support offsets -- no_llseek is the in-tree
	 * convention for that (e.g. drivers/spi/spidev.c), preventing
	 * unexpected seek behavior on the default llseek. Found by Copilot's
	 * PR #46 review.
	 */
	.llseek = no_llseek,
};

static ssize_t link_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct mt_transport_priv *priv = spi_get_drvdata(spi);
	const char *state;

	(void)attr;
	switch (spiTransportGetLinkState(priv->htransport)) {
	case eSpiTransportLinkConnected:
		state = "connected";
		break;
	case eSpiTransportLinkHandshaking:
		state = "handshaking";
		break;
	default:
		state = "disconnected";
		break;
	}
	return sysfs_emit(buf, "%s\n", state);
}
static DEVICE_ATTR_RO(link_state);

static ssize_t event_counters_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct mt_transport_priv *priv = spi_get_drvdata(spi);

	(void)attr;
	/* Field names match the STM32 Client harness's [DBG] line
	 * (conn=/disc=/hdrCrc=/payCrc=/seq=/dmaFail=/dmaTo=) so a
	 * fault-injection run's peer-side verdict can be read off this file
	 * directly against that harness's docs/TestPlan.md.
	 */
	return sysfs_emit(buf, "conn=%d disc=%d hdrCrc=%d payCrc=%d seq=%d dmaFail=%d dmaTo=%d\n",
			   atomic_read(&priv->evt_connected), atomic_read(&priv->evt_disconnected),
			   atomic_read(&priv->evt_hdr_crc), atomic_read(&priv->evt_payload_crc),
			   atomic_read(&priv->evt_seq_gap), atomic_read(&priv->evt_dma_failure),
			   atomic_read(&priv->evt_dma_timeout));
}
static DEVICE_ATTR_RO(event_counters);

static struct attribute *mt_transport_attrs[] = {
	&dev_attr_link_state.attr,
	&dev_attr_event_counters.attr,
	NULL,
};
ATTRIBUTE_GROUPS(mt_transport);

static int mt_transport_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct mt_transport_priv *priv;
	trSpiTransportConfig config;
	int ret;
	int i;
	int started_misc = 0;

	/* Plain kzalloc, not devm_kzalloc: see the refcount/available/removed
	 * comment on struct mt_transport_priv -- this memory must be able to
	 * outlive the SPI device's unbind if userspace still holds a
	 * channel's device open.
	 */
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	kref_init(&priv->refcount);

	priv->spi = spi;
	priv->dev = dev;
	spi_set_drvdata(spi, priv);

	for (i = 0; i < MT_TRANSPORT_NUM_CHANNELS; i++) {
		struct mt_transport_channel *chan = &priv->channels[i];

		chan->priv = priv;
		chan->channel_num = (uint8_t)(i + 1);
		if (chan->channel_num == 1)
			strscpy(chan->misc_name, "mt_spi_transport", sizeof(chan->misc_name));
		else
			scnprintf(chan->misc_name, sizeof(chan->misc_name), "mt_spi_transport_ch%u",
				  chan->channel_num);
		atomic_set(&chan->available, 1);
		init_waitqueue_head(&chan->rx_wq);
		spin_lock_init(&chan->rx_lock);
		spin_lock_init(&chan->tx_lock);
		init_waitqueue_head(&chan->tx_free_wq);
		chan->tx_in_flight_idx = -1;
	}

	/* Custom "multitracks,nss"/"multitracks,nrdy" bindings, not the
	 * standard "cs-gpios" -- see spi_transport_hw_linux.c's file comment
	 * for why these must stay outside the SPI core's own chip-select
	 * handling. Vendor-prefixed per the DT binding's own
	 * multitracks,spi-transport.yaml (and the "multitracks" registry
	 * entry in vendor-prefixes.yaml) -- devm_gpiod_get()'s con_id here
	 * has the "-gpios" suffix implicitly appended to form the property
	 * name it looks up, so "multitracks,nss" resolves to the
	 * "multitracks,nss-gpios" DT property. Found by Copilot's PR #46
	 * review.
	 */
	priv->nss_gpiod = devm_gpiod_get(dev, "multitracks,nss", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->nss_gpiod)) {
		ret = PTR_ERR(priv->nss_gpiod);
		kref_put(&priv->refcount, mt_transport_priv_release);
		return dev_err_probe(dev, ret, "failed to get multitracks,nss-gpios\n");
	}

	priv->nrdy_gpiod = devm_gpiod_get(dev, "multitracks,nrdy", GPIOD_IN);
	if (IS_ERR(priv->nrdy_gpiod)) {
		ret = PTR_ERR(priv->nrdy_gpiod);
		kref_put(&priv->refcount, mt_transport_priv_release);
		return dev_err_probe(dev, ret, "failed to get multitracks,nrdy-gpios\n");
	}

	ret = mt_transport_os_linux_init(&priv->os_ctx, dev, &priv->os);
	if (ret) {
		kref_put(&priv->refcount, mt_transport_priv_release);
		return dev_err_probe(dev, ret, "mt_transport_os_linux_init failed\n");
	}
	mt_transport_hw_linux_init(&priv->hw_ctx, spi, priv->nss_gpiod, priv->nrdy_gpiod, &priv->hw);
	mt_transport_hw_linux_set_notify(&priv->hw_ctx, mt_transport_tick_notify, priv);

	config.role = eSpiTransportRoleHost;
	config.prOs = &priv->os;
	config.prHw = &priv->hw;

	if (spiTransportInit(&config, &priv->htransport) != eSpiTransportErrorNone) {
		kref_put(&priv->refcount, mt_transport_priv_release);
		return dev_err_probe(dev, -EINVAL, "spiTransportInit failed\n");
	}

	for (i = 0; i < MT_TRANSPORT_NUM_CHANNELS; i++) {
		struct mt_transport_channel *chan = &priv->channels[i];

		if (spiTransportRegisterChannel(priv->htransport, chan->channel_num,
						 mt_transport_rx_callback, mt_transport_event_callback,
						 chan)
		    != eSpiTransportErrorNone) {
			kref_put(&priv->refcount, mt_transport_priv_release);
			return dev_err_probe(dev, -EINVAL,
					     "spiTransportRegisterChannel(%u) failed\n",
					     chan->channel_num);
		}
	}

	/* Optional latency optimization -- if the NRDY line has no usable
	 * IRQ, tick-driven pReadyRead() polling (nominally every 2ms, see the
	 * tick thread above -- actually whatever HZ rounds msecs_to_jiffies(2)
	 * up to, e.g. 4ms or 10ms depending on kernel config) is a fully
	 * sufficient fallback per the core's own contract, so a failure here
	 * is not fatal.
	 */
	priv->nrdy_irq = gpiod_to_irq(priv->nrdy_gpiod);
	/* -EPROBE_DEFER means the IRQ chip backing this GPIO isn't ready yet,
	 * not "this GPIO has no IRQ" -- must propagate it so the kernel
	 * retries this whole probe() later, or the IRQ optimization gets
	 * silently and permanently disabled by a boot-time ordering race
	 * instead of the actual IRQ becoming available a bit later.
	 */
	if (priv->nrdy_irq == -EPROBE_DEFER) {
		kref_put(&priv->refcount, mt_transport_priv_release);
		return dev_err_probe(dev, -EPROBE_DEFER, "NRDY IRQ not ready yet\n");
	}
	/* >= 0, not > 0: IRQ 0 is a legally valid IRQ number on some
	 * platforms/irqdomains -- only negative values mean "no IRQ" per
	 * gpiod_to_irq()'s own contract. Treating 0 as "no IRQ" would skip
	 * the threaded handler for a GPIO that genuinely maps to IRQ 0.
	 */
	if (priv->nrdy_irq >= 0) {
		ret = devm_request_threaded_irq(dev, priv->nrdy_irq, NULL,
						 mt_transport_hw_linux_nrdy_irq,
						 IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
							 | IRQF_ONESHOT,
						 DRIVER_NAME "-nrdy", &priv->hw_ctx);
		if (ret)
			dev_dbg(dev, "no NRDY IRQ (%d) -- falling back to tick-poll only\n", ret);
		else
			priv->nrdy_irq_requested = true;
	} else {
		dev_dbg(dev, "NRDY line has no IRQ -- tick-poll only\n");
	}

	priv->tick_thread = kthread_run(mt_transport_tick_thread_fn, priv, "%s-tick", DRIVER_NAME);
	if (IS_ERR(priv->tick_thread)) {
		ret = PTR_ERR(priv->tick_thread);
		/* The NRDY IRQ may already be live at this point (it's
		 * requested earlier in this function) -- unlike devm_kzalloc,
		 * kref_put() below can free priv's memory (including the
		 * embedded hw_ctx the IRQ handler dereferences) synchronously,
		 * right now, not deferred to the same devm unwind pass that
		 * would otherwise free the IRQ first. Must free it explicitly
		 * before dropping the last reference, same as
		 * mt_transport_remove() does. Found by Copilot's PR #46
		 * review (on the two later error paths below; this one has
		 * the identical bug since the IRQ is requested even earlier).
		 */
		if (priv->nrdy_irq_requested)
			devm_free_irq(dev, priv->nrdy_irq, &priv->hw_ctx);
		kref_put(&priv->refcount, mt_transport_priv_release);
		return dev_err_probe(dev, ret, "failed to start tick thread\n");
	}

	if (spiTransportStart(priv->htransport) != eSpiTransportErrorNone) {
		/* Free the IRQ before stopping the tick thread -- see
		 * mt_transport_remove()'s comment: the threaded handler can
		 * still fire and call pOnReadyEvent() into a transport that's
		 * mid-teardown otherwise. Found by Copilot's PR #46 review
		 * (flagged on the misc_register() failure path below; this
		 * path has the identical ordering bug).
		 */
		if (priv->nrdy_irq_requested)
			devm_free_irq(dev, priv->nrdy_irq, &priv->hw_ctx);
		kthread_stop(priv->tick_thread);
		kref_put(&priv->refcount, mt_transport_priv_release);
		return dev_err_probe(dev, -EINVAL, "spiTransportStart failed\n");
	}

	for (i = 0; i < MT_TRANSPORT_NUM_CHANNELS; i++) {
		struct mt_transport_channel *chan = &priv->channels[i];

		chan->misc.minor = MISC_DYNAMIC_MINOR;
		chan->misc.name = chan->misc_name;
		chan->misc.fops = &mt_transport_misc_fops;
		ret = misc_register(&chan->misc);
		if (ret) {
			int j;

			/* Free the IRQ first, matching mt_transport_remove()'s
			 * ordering -- the threaded handler can still fire and
			 * call pOnReadyEvent() into a transport that's
			 * mid-teardown otherwise. Found by Copilot's PR #46
			 * review.
			 */
			if (priv->nrdy_irq_requested)
				devm_free_irq(dev, priv->nrdy_irq, &priv->hw_ctx);
			/* An earlier channel's misc_register() already
			 * succeeded, so userspace can already hold an fd on
			 * it by the time a later channel's registration fails
			 * here -- set removed and wake its wait queues first,
			 * same as mt_transport_remove(), or a read()/write()
			 * already blocked on that fd would sleep forever past
			 * the misc_deregister() below. Found by Copilot's PR
			 * #46 review.
			 */
			WRITE_ONCE(priv->removed, true);
			for (j = 0; j < started_misc; j++) {
				wake_up_interruptible_all(&priv->channels[j].rx_wq);
				wake_up_interruptible_all(&priv->channels[j].tx_free_wq);
			}
			/* Roll back any channel's misc device already
			 * registered before this one failed -- otherwise a
			 * second channel's registration failure would leak
			 * the first channel's /dev node across this probe
			 * failure.
			 */
			for (j = 0; j < started_misc; j++)
				misc_deregister(&priv->channels[j].misc);
			kthread_stop(priv->tick_thread);
			spiTransportStop(priv->htransport);
			kref_put(&priv->refcount, mt_transport_priv_release);
			return dev_err_probe(dev, ret, "misc_register(%s) failed\n",
					     chan->misc_name);
		}
		started_misc++;
	}

	dev_info(dev, "MultiTracks SPI transport driver probed (Host role, %d channels)\n",
		 MT_TRANSPORT_NUM_CHANNELS);
	return 0;
}

static void mt_transport_remove(struct spi_device *spi)
{
	struct mt_transport_priv *priv = spi_get_drvdata(spi);
	int i;

	/* Set first, before anything else is torn down: any read()/write()
	 * on an fd that outlived this unbind must bail out with -ENODEV
	 * instead of touching the os/hw state this function is about to
	 * stop -- e.g. write()'s priv->os.pTaskNotifyGive() call would
	 * otherwise reach into a tick thread/os_ctx that's mid-teardown or
	 * already gone. This is a fail-fast measure, not the memory-safety
	 * fix itself -- priv's memory itself stays valid for as long as any
	 * such fd remains open via the kref below (see struct
	 * mt_transport_priv's refcount/removed comment). Found by
	 * Copilot's PR #46 review.
	 */
	WRITE_ONCE(priv->removed, true);
	/* Wake any reader/writer already blocked in wait_event_interruptible()
	 * on either channel's rx_wq/tx_free_wq -- both now include removed in
	 * their wait condition, but a waiter sleeping before this
	 * WRITE_ONCE() would otherwise never re-check it and could hang
	 * forever across unbind. Found by Copilot's PR #46 review.
	 */
	for (i = 0; i < MT_TRANSPORT_NUM_CHANNELS; i++) {
		wake_up_interruptible_all(&priv->channels[i].rx_wq);
		wake_up_interruptible_all(&priv->channels[i].tx_free_wq);
	}

	/* Free the NRDY IRQ (if one was ever successfully requested) before
	 * anything else: it's devm-managed, so it would otherwise only get
	 * freed by the driver core *after* this function returns, leaving it
	 * live through misc_deregister()/kthread_stop()/spiTransportStop()
	 * below -- if the line toggles during that window (a real STM32
	 * Client peer doesn't stop just because this side is unbinding),
	 * mt_transport_hw_linux_nrdy_irq() could fire and call
	 * priv->hw.pOnReadyEvent() into a core that's mid-teardown.
	 * devm_free_irq() blocks until any already-running (threaded)
	 * handler invocation finishes, so this also can't race an
	 * in-flight callback the way a bare disable_irq() wouldn't.
	 */
	if (priv->nrdy_irq_requested)
		devm_free_irq(priv->dev, priv->nrdy_irq, &priv->hw_ctx);

	for (i = 0; i < MT_TRANSPORT_NUM_CHANNELS; i++)
		misc_deregister(&priv->channels[i].misc);
	/* kthread_stop() blocks until the tick thread's loop actually exits,
	 * guaranteeing no thread is still calling spiTransportTick()/
	 * mt_transport_tx_service() by the time spiTransportStop() runs.
	 * The reverse order (stop-then-kthread_stop, the original ordering
	 * here) left a window where the still-running tick thread could call
	 * into the core after it was already torn down.
	 */
	kthread_stop(priv->tick_thread);
	spiTransportStop(priv->htransport);

	/* Drops the driver instance's own reference (taken via kref_init()
	 * in probe()). If no fd is currently open on either channel, this is
	 * the last reference and priv is freed right here -- same timing as
	 * the devm-based lifetime this replaced. If userspace still holds
	 * either channel's device open, priv stays alive (kept by that fd's
	 * own reference from mt_transport_misc_open()) until it's closed.
	 */
	kref_put(&priv->refcount, mt_transport_priv_release);
}

static const struct of_device_id mt_transport_of_match[] = {
	{ .compatible = "multitracks,spi-transport", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt_transport_of_match);

static struct spi_driver mt_transport_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = mt_transport_of_match,
		.dev_groups = mt_transport_groups,
	},
	.probe = mt_transport_probe,
	.remove = mt_transport_remove,
};
module_spi_driver(mt_transport_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("MultiTracks.com, LLC.");
MODULE_DESCRIPTION("MultiTracks SPI transport protocol driver (Host role)");
