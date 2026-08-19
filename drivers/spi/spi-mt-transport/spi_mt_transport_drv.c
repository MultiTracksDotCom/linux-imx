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
 * below (misc device + a small TX ring, see mt_transport_tx_service())
 * is still a placeholder ahead of MT-158682's real design: single
 * hardcoded channel, single in-flight RX message, no ioctl/config surface.
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

#include "spi_transport/spi_transport.h"
#include "spi_transport/spi_transport_types.h"
#include "spi_transport_os_linux.h"
#include "spi_transport_hw_linux.h"

#define DRIVER_NAME "spi-mt-transport"
#define MT_TRANSPORT_CHANNEL 1
#define MT_TRANSPORT_TX_QUEUE_DEPTH 10

struct mt_transport_tx_slot
{
	uint8_t buf[SPI_TRANSPORT_CHANNEL_MESSAGE_MAX];
	uint16_t len;
};

struct mt_transport_priv {
	struct spi_device *spi;
	struct device *dev;

	struct gpio_desc *nss_gpiod;
	struct gpio_desc *nrdy_gpiod;
	int nrdy_irq;

	trSpiTransportOs os;
	trSpiTransportHw hw;
	struct mt_transport_os_ctx os_ctx;
	struct mt_transport_hw_ctx hw_ctx;
	thSpiTransport htransport;

	struct task_struct *tick_thread;

	/* Minimal placeholder userspace interface -- MT-158682 owns the real
	 * design. Single hardcoded channel, single in-flight RX message,
	 * blocking read()/write(), best-effort poll().
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
	int tx_in_flight_idx; /* -1 if nothing submitted yet */
	wait_queue_head_t tx_free_wq;

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
	 * read from arbitrary userspace context via sysfs.
	 */
	atomic_t evt_connected;
	atomic_t evt_disconnected;
	atomic_t evt_hdr_crc;
	atomic_t evt_payload_crc;
	atomic_t evt_seq_gap;
	atomic_t evt_dma_failure;
	atomic_t evt_dma_timeout;
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
	struct mt_transport_priv *priv = pContext;
	unsigned long irqflags;

	(void)flags;
	if (channel != MT_TRANSPORT_CHANNEL)
		return;
	if (length > sizeof(priv->rx_buf))
		length = sizeof(priv->rx_buf);

	spin_lock_irqsave(&priv->rx_lock, irqflags);
	memcpy(priv->rx_buf, pBuffer, length);
	priv->rx_len = length;
	/* WRITE_ONCE() pairs with the unlocked READ_ONCE() reads of rx_valid
	 * in mt_transport_misc_poll() and the wait_event_interruptible()
	 * condition below -- rx_buf/rx_len are only ever touched under
	 * rx_lock, but rx_valid itself is also read lock-free in those two
	 * spots (both are the standard Linux poll_wait()/wait_event idiom,
	 * where the wait/wake primitives themselves provide the needed
	 * ordering -- this is about being explicit for readers/tooling like
	 * KCSAN, not fixing an actual race).
	 */
	WRITE_ONCE(priv->rx_valid, true);
	spin_unlock_irqrestore(&priv->rx_lock, irqflags);

	wake_up_interruptible(&priv->rx_wq);
}

static void mt_transport_event_callback(void *pContext, teSpiTransportEvent eEvent)
{
	struct mt_transport_priv *priv = pContext;

	switch (eEvent) {
	case eSpiTransportEventConnected:
		atomic_inc(&priv->evt_connected);
		dev_info(priv->dev, "link event: connected\n");
		break;
	case eSpiTransportEventDisconnected:
		atomic_inc(&priv->evt_disconnected);
		dev_info(priv->dev, "link event: disconnected\n");
		break;
	case eSpiTransportEventErrorHeaderCrc:
		atomic_inc(&priv->evt_hdr_crc);
		dev_warn(priv->dev, "link event: header CRC error\n");
		break;
	case eSpiTransportEventErrorPayloadCrc:
		atomic_inc(&priv->evt_payload_crc);
		dev_warn(priv->dev, "link event: payload CRC error\n");
		break;
	case eSpiTransportEventErrorSequenceGap:
		atomic_inc(&priv->evt_seq_gap);
		dev_warn(priv->dev, "link event: sequence gap\n");
		break;
	case eSpiTransportEventErrorDmaFailure:
		atomic_inc(&priv->evt_dma_failure);
		dev_warn(priv->dev, "link event: DMA arm failure\n");
		break;
	case eSpiTransportEventErrorDmaTimeout:
		atomic_inc(&priv->evt_dma_timeout);
		dev_warn(priv->dev, "link event: DMA timeout\n");
		break;
	default:
		dev_warn(priv->dev, "link event: unknown (%d)\n", (int)eEvent);
		break;
	}
}

/// @brief Submit the oldest queued TX slot (if any) via spiTransportSend().
///        A success return proves the *previous* in-flight slot (if any) is
///        now done -- the core only accepts a new send once the last one's
///        final chunk is confirmed -- so that previous slot is freed right
///        here, not after a guessed timeout. Called once per tick thread
///        iteration; a Busy return just means retry next tick, no state
///        changes.
///
/// Gates on tx_queued_count, not "is anything occupied at all": an earlier
/// version checked the combined queued+in-flight total, which let tx_head
/// advance onto a slot write() had never actually filled whenever exactly
/// one message was in flight and nothing new had been queued behind it --
/// tx_service() would then resend whatever stale bytes happened to be
/// sitting in that slot (found via Copilot PR review). Tracking queued
/// count separately from "is one slot in flight" makes "is there anything
/// NEW to submit" the only thing this check needs to answer.
static void mt_transport_tx_service(struct mt_transport_priv *priv)
{
	unsigned long flags;
	unsigned int idx;
	uint16_t len;
	teSpiTransportError err;

	spin_lock_irqsave(&priv->tx_lock, flags);
	if (priv->tx_queued_count == 0) {
		spin_unlock_irqrestore(&priv->tx_lock, flags);
		return;
	}
	idx = priv->tx_head;
	len = priv->tx_slots[idx].len;
	spin_unlock_irqrestore(&priv->tx_lock, flags);

	err = spiTransportSend(priv->htransport, MT_TRANSPORT_CHANNEL, priv->tx_slots[idx].buf, len,
				true);
	if (err != eSpiTransportErrorNone)
		return;

	spin_lock_irqsave(&priv->tx_lock, flags);
	priv->tx_queued_count--;
	priv->tx_in_flight_idx = idx;
	priv->tx_head = (priv->tx_head + 1) % MT_TRANSPORT_TX_QUEUE_DEPTH;
	spin_unlock_irqrestore(&priv->tx_lock, flags);
	wake_up_interruptible(&priv->tx_free_wq);
}

static int mt_transport_tick_thread_fn(void *data)
{
	struct mt_transport_priv *priv = data;

	while (!kthread_should_stop()) {
		priv->os.pTaskNotifyWait(priv->os.pContext, 2);
		spiTransportTick(priv->htransport);
		mt_transport_tx_service(priv);
	}
	return 0;
}

/* --- Minimal userspace interface (placeholder ahead of MT-158682) --- */

static ssize_t mt_transport_misc_read(struct file *filp, char __user *buf, size_t count,
				       loff_t *ppos)
{
	struct miscdevice *misc = filp->private_data;
	struct mt_transport_priv *priv = container_of(misc, struct mt_transport_priv, misc);
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

	if (filp->f_flags & O_NONBLOCK) {
		spin_lock_irqsave(&priv->rx_lock, irqflags);
		if (!priv->rx_valid) {
			spin_unlock_irqrestore(&priv->rx_lock, irqflags);
			return -EAGAIN;
		}
		spin_unlock_irqrestore(&priv->rx_lock, irqflags);
	} else {
		ret = wait_event_interruptible(priv->rx_wq, READ_ONCE(priv->rx_valid));
		if (ret)
			return ret;
	}

	/* Snapshot into a local buffer under the lock, then copy_to_user()
	 * outside it -- copy_to_user() can fault/sleep, which is illegal
	 * while holding a spinlock. rx_valid is cleared here too (not after
	 * the copy) since it's rx_lock-protected state, same as rx_buf --
	 * a failing copy_to_user (a broken caller's bad pointer) now
	 * consumes the buffered message rather than leaving it for retry,
	 * a minor, acceptable behavior change for this placeholder interface.
	 */
	spin_lock_irqsave(&priv->rx_lock, irqflags);
	len = priv->rx_len;
	if (len > count)
		len = count;
	memcpy(scratch, priv->rx_buf, len);
	WRITE_ONCE(priv->rx_valid, false);
	spin_unlock_irqrestore(&priv->rx_lock, irqflags);

	if (copy_to_user(buf, scratch, len))
		return -EFAULT;

	return len;
}

/// @brief True if a new slot can be enqueued. Caller must already hold
///        tx_lock -- occupied total is tx_queued_count (not-yet-submitted
///        slots) plus one more if a slot is currently in flight
///        (tx_in_flight_idx >= 0), since that slot is still reserved even
///        though it doesn't count toward tx_queued_count.
static inline bool mt_transport_tx_room_locked(struct mt_transport_priv *priv)
{
	unsigned int occupied = priv->tx_queued_count + (priv->tx_in_flight_idx >= 0 ? 1 : 0);

	return occupied < MT_TRANSPORT_TX_QUEUE_DEPTH;
}

/// @brief wait_event_interruptible()'s condition check only -- takes and
///        releases tx_lock itself since it must be callable without
///        already holding it. mt_transport_misc_write()'s own room check
///        below calls mt_transport_tx_room_locked() directly instead
///        (already holding the lock at that point) rather than this
///        wrapper, and deliberately so: that check has to stay under the
///        *same* lock acquisition that immediately follows (the enqueue),
///        otherwise a second writer could take the now-free slot in the
///        gap between checking and re-locking.
static bool mt_transport_tx_has_room(struct mt_transport_priv *priv)
{
	unsigned long flags;
	bool room;

	spin_lock_irqsave(&priv->tx_lock, flags);
	room = mt_transport_tx_room_locked(priv);
	spin_unlock_irqrestore(&priv->tx_lock, flags);
	return room;
}

static ssize_t mt_transport_misc_write(struct file *filp, const char __user *buf, size_t count,
					loff_t *ppos)
{
	struct miscdevice *misc = filp->private_data;
	struct mt_transport_priv *priv = container_of(misc, struct mt_transport_priv, misc);
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

	if (len > sizeof(scratch))
		len = sizeof(scratch);
	if (copy_from_user(scratch, buf, len))
		return -EFAULT;

	for (;;) {
		spin_lock_irqsave(&priv->tx_lock, flags);
		if (mt_transport_tx_room_locked(priv))
			break;
		spin_unlock_irqrestore(&priv->tx_lock, flags);

		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		ret = wait_event_interruptible(priv->tx_free_wq, mt_transport_tx_has_room(priv));
		if (ret)
			return ret;
	}

	idx = priv->tx_tail;
	memcpy(priv->tx_slots[idx].buf, scratch, len);
	priv->tx_slots[idx].len = len;
	priv->tx_tail = (priv->tx_tail + 1) % MT_TRANSPORT_TX_QUEUE_DEPTH;
	priv->tx_queued_count++;
	spin_unlock_irqrestore(&priv->tx_lock, flags);

	/* Kick the tick thread so mt_transport_tx_service() attempts this
	 * send right away instead of waiting up to its ~2ms poll interval.
	 */
	priv->os.pTaskNotifyGive(priv->os.pContext);

	return len;
}

static __poll_t mt_transport_misc_poll(struct file *filp, poll_table *wait)
{
	struct miscdevice *misc = filp->private_data;
	struct mt_transport_priv *priv = container_of(misc, struct mt_transport_priv, misc);
	__poll_t mask = 0;

	poll_wait(filp, &priv->rx_wq, wait);
	if (READ_ONCE(priv->rx_valid))
		mask |= EPOLLIN | EPOLLRDNORM;
	return mask;
}

static const struct file_operations mt_transport_misc_fops = {
	.owner = THIS_MODULE,
	.read = mt_transport_misc_read,
	.write = mt_transport_misc_write,
	.poll = mt_transport_misc_poll,
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

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->spi = spi;
	priv->dev = dev;
	spi_set_drvdata(spi, priv);

	init_waitqueue_head(&priv->rx_wq);
	spin_lock_init(&priv->rx_lock);
	spin_lock_init(&priv->tx_lock);
	init_waitqueue_head(&priv->tx_free_wq);
	priv->tx_in_flight_idx = -1;

	/* Custom "mt-nss"/"mt-nrdy" bindings, not the standard "cs-gpios" --
	 * see spi_transport_hw_linux.c's file comment for why these must stay
	 * outside the SPI core's own chip-select handling.
	 */
	priv->nss_gpiod = devm_gpiod_get(dev, "mt-nss", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->nss_gpiod))
		return dev_err_probe(dev, PTR_ERR(priv->nss_gpiod),
				      "failed to get mt-nss-gpios\n");

	priv->nrdy_gpiod = devm_gpiod_get(dev, "mt-nrdy", GPIOD_IN);
	if (IS_ERR(priv->nrdy_gpiod))
		return dev_err_probe(dev, PTR_ERR(priv->nrdy_gpiod),
				      "failed to get mt-nrdy-gpios\n");

	ret = mt_transport_os_linux_init(&priv->os_ctx, dev, &priv->os);
	if (ret)
		return dev_err_probe(dev, ret, "mt_transport_os_linux_init failed\n");
	mt_transport_hw_linux_init(&priv->hw_ctx, spi, priv->nss_gpiod, priv->nrdy_gpiod, &priv->hw);
	mt_transport_hw_linux_set_notify(&priv->hw_ctx, mt_transport_tick_notify, priv);

	config.role = eSpiTransportRoleHost;
	config.prOs = &priv->os;
	config.prHw = &priv->hw;

	if (spiTransportInit(&config, &priv->htransport) != eSpiTransportErrorNone)
		return dev_err_probe(dev, -EINVAL, "spiTransportInit failed\n");

	if (spiTransportRegisterChannel(priv->htransport, MT_TRANSPORT_CHANNEL,
					 mt_transport_rx_callback, mt_transport_event_callback,
					 priv)
	    != eSpiTransportErrorNone)
		return dev_err_probe(dev, -EINVAL, "spiTransportRegisterChannel failed\n");

	/* Optional latency optimization -- if the NRDY line has no usable
	 * IRQ, tick-driven pReadyRead() polling (nominally every 2ms, see the
	 * tick thread above -- actually whatever HZ rounds msecs_to_jiffies(2)
	 * up to, e.g. 4ms or 10ms depending on kernel config) is a fully
	 * sufficient fallback per the core's own contract, so a failure here
	 * is not fatal.
	 */
	priv->nrdy_irq = gpiod_to_irq(priv->nrdy_gpiod);
	if (priv->nrdy_irq > 0) {
		ret = devm_request_threaded_irq(dev, priv->nrdy_irq, NULL,
						 mt_transport_hw_linux_nrdy_irq,
						 IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
							 | IRQF_ONESHOT,
						 DRIVER_NAME "-nrdy", &priv->hw_ctx);
		if (ret)
			dev_dbg(dev, "no NRDY IRQ (%d) -- falling back to tick-poll only\n", ret);
	} else {
		dev_dbg(dev, "NRDY line has no IRQ -- tick-poll only\n");
	}

	priv->tick_thread = kthread_run(mt_transport_tick_thread_fn, priv, "%s-tick", DRIVER_NAME);
	if (IS_ERR(priv->tick_thread))
		return dev_err_probe(dev, PTR_ERR(priv->tick_thread),
				      "failed to start tick thread\n");

	if (spiTransportStart(priv->htransport) != eSpiTransportErrorNone) {
		kthread_stop(priv->tick_thread);
		return dev_err_probe(dev, -EINVAL, "spiTransportStart failed\n");
	}

	/* Name hardcoded, not suffixed per-device (flagged by Copilot review --
	 * see PR discussion): a second bound spi-mt-transport device would
	 * collide here, but this specific hardware only ever binds one, and
	 * changing the path breaks every existing script/tool this session
	 * built against /dev/mt_spi_transport. Left as-is pending a decision;
	 * see PR #785.
	 */
	priv->misc.minor = MISC_DYNAMIC_MINOR;
	priv->misc.name = "mt_spi_transport";
	priv->misc.fops = &mt_transport_misc_fops;
	ret = misc_register(&priv->misc);
	if (ret) {
		/* kthread_stop() before spiTransportStop(): the tick thread
		 * must not be able to call into the core after it's been
		 * stopped (same ordering as mt_transport_remove() below).
		 */
		kthread_stop(priv->tick_thread);
		spiTransportStop(priv->htransport);
		return dev_err_probe(dev, ret, "misc_register failed\n");
	}

	dev_info(dev, "MultiTracks SPI transport driver probed (Host role)\n");
	return 0;
}

static void mt_transport_remove(struct spi_device *spi)
{
	struct mt_transport_priv *priv = spi_get_drvdata(spi);

	misc_deregister(&priv->misc);
	/* kthread_stop() blocks until the tick thread's loop actually exits,
	 * guaranteeing no thread is still calling spiTransportTick()/
	 * mt_transport_tx_service() by the time spiTransportStop() runs.
	 * The reverse order (stop-then-kthread_stop, the original ordering
	 * here) left a window where the still-running tick thread could call
	 * into the core after it was already torn down.
	 */
	kthread_stop(priv->tick_thread);
	spiTransportStop(priv->htransport);
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
