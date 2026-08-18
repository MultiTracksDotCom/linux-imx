// SPDX-License-Identifier: GPL-2.0
/*
 * spi_mt_transport_drv.c - Linux kernel Host-role driver for the MultiTracks
 * SPI transport protocol (see core/ for the portable protocol core, vendored
 * from the firmware repo -- see core/PROVENANCE.md).
 *
 * Talks Host role to an STM32-class Client peer over a raw SPI bus plus a
 * companion NRDY GPIO handshake line. See the firmware repo
 * (firmware-common/spi-transport/docs/ProtocolSpec.md) for the wire protocol
 * and handshake state machine this ports into the kernel.
 *
 * Scope note (MT-158113): this is the driver only. The EVK-side test
 * framework (MT-158682) is a separate ticket -- the userspace interface
 * below is a deliberately minimal placeholder, just enough to prove the
 * module loads, probes, and can move a byte.
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
	priv->rx_valid = true;
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

static int mt_transport_tick_thread_fn(void *data)
{
	struct mt_transport_priv *priv = data;

	while (!kthread_should_stop()) {
		priv->os.pTaskNotifyWait(priv->os.pContext, 2);
		spiTransportTick(priv->htransport);
	}
	return 0;
}

/* --- Minimal userspace interface (placeholder, see plan sec 7) --- */

static ssize_t mt_transport_misc_read(struct file *filp, char __user *buf, size_t count,
				       loff_t *ppos)
{
	struct miscdevice *misc = filp->private_data;
	struct mt_transport_priv *priv = container_of(misc, struct mt_transport_priv, misc);
	unsigned long irqflags;
	uint16_t len;
	int ret;

	(void)ppos;

	if (filp->f_flags & O_NONBLOCK) {
		spin_lock_irqsave(&priv->rx_lock, irqflags);
		if (!priv->rx_valid) {
			spin_unlock_irqrestore(&priv->rx_lock, irqflags);
			return -EAGAIN;
		}
		spin_unlock_irqrestore(&priv->rx_lock, irqflags);
	} else {
		ret = wait_event_interruptible(priv->rx_wq, priv->rx_valid);
		if (ret)
			return ret;
	}

	spin_lock_irqsave(&priv->rx_lock, irqflags);
	len = priv->rx_len;
	if (len > count)
		len = count;
	if (copy_to_user(buf, priv->rx_buf, len)) {
		spin_unlock_irqrestore(&priv->rx_lock, irqflags);
		return -EFAULT;
	}
	priv->rx_valid = false;
	spin_unlock_irqrestore(&priv->rx_lock, irqflags);

	return len;
}

static ssize_t mt_transport_misc_write(struct file *filp, const char __user *buf, size_t count,
					loff_t *ppos)
{
	struct miscdevice *misc = filp->private_data;
	struct mt_transport_priv *priv = container_of(misc, struct mt_transport_priv, misc);
	uint8_t txBuf[SPI_TRANSPORT_CHANNEL_MESSAGE_MAX];
	teSpiTransportError err;
	size_t len = count;

	(void)ppos;

	if (len > sizeof(txBuf))
		len = sizeof(txBuf);
	if (copy_from_user(txBuf, buf, len))
		return -EFAULT;

	err = spiTransportSend(priv->htransport, MT_TRANSPORT_CHANNEL, txBuf, len, true);
	if (err != eSpiTransportErrorNone)
		return -EBUSY;

	return len;
}

static __poll_t mt_transport_misc_poll(struct file *filp, poll_table *wait)
{
	struct miscdevice *misc = filp->private_data;
	struct mt_transport_priv *priv = container_of(misc, struct mt_transport_priv, misc);
	__poll_t mask = 0;

	poll_wait(filp, &priv->rx_wq, wait);
	if (priv->rx_valid)
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

	mt_transport_os_linux_init(&priv->os_ctx, dev, &priv->os);
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
	 * IRQ, tick-driven pReadyRead() polling (2ms cadence, see the tick
	 * thread above) is a fully sufficient fallback per the core's own
	 * contract, so a failure here is not fatal.
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

	priv->misc.minor = MISC_DYNAMIC_MINOR;
	priv->misc.name = "mt_spi_transport";
	priv->misc.fops = &mt_transport_misc_fops;
	ret = misc_register(&priv->misc);
	if (ret) {
		spiTransportStop(priv->htransport);
		kthread_stop(priv->tick_thread);
		return dev_err_probe(dev, ret, "misc_register failed\n");
	}

	dev_info(dev, "MultiTracks SPI transport driver probed (Host role)\n");
	return 0;
}

static void mt_transport_remove(struct spi_device *spi)
{
	struct mt_transport_priv *priv = spi_get_drvdata(spi);

	misc_deregister(&priv->misc);
	spiTransportStop(priv->htransport);
	kthread_stop(priv->tick_thread);
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
