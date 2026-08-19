/* SPDX-License-Identifier: GPL-2.0 */
/*
 * spi_transport_hw_linux.h - Linux kernel HW-adapter for the MultiTracks SPI
 * transport core, Host role only (see
 * ../../spi_transport_hw.h for the contract).
 *
 * Host role does not need, and must not wire, pReadyAssert (Client-only,
 * drives NRDY), pOnSelectEvent (Client-only, watches for an edge on a pin
 * only Host itself drives) or pOnClockStart (Client-only "clocking started"
 * latch) -- see spi_mt_transport_drv.c's probe() for where this is asserted.
 */

#ifndef SPI_TRANSPORT_HW_LINUX_H
#define SPI_TRANSPORT_HW_LINUX_H

#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/completion.h>

#include "spi_transport/spi_transport_hw.h"

struct mt_transport_hw_ctx {
	struct spi_device *spi;
	struct gpio_desc *nss_gpiod;  /* Host-driven request/select line */
	struct gpio_desc *nrdy_gpiod; /* Host reads only; Client drives it */

	/* Back-pointer to the trSpiTransportHw instance this ctx is
	 * pContext for -- spiTransportHwSetCallbacks() (called by the core
	 * during spiTransportInit()) fills in pOnTransferComplete/
	 * pOnReadyEvent/pCoreCtx directly on *this* struct, not on ctx, so
	 * the completion/IRQ paths reach them through here.
	 */
	trSpiTransportHw *pHw;

	/* Reused across every transfer -- the core's Host state machine only
	 * ever has one transfer in flight at a time. That invariant is
	 * enforced (not just assumed) via transferComplete: "done" means no
	 * spi_async() is outstanding against msg/xfer, so it's safe to
	 * reinitialize them. Without this, mt_hw_abort() being a no-op could
	 * let a retry reinitialize msg/xfer while the SPI core still had the
	 * previous submission queued/in-flight, corrupting its internal
	 * message-queue and scatterlist state -- see the NULL-deref crash in
	 * spi_imx_dma_transfer()'s sg_last() this was written to fix.
	 */
	struct spi_message msg;
	struct spi_transfer xfer;
	struct completion transferComplete;

	/* Completion notify to wake the driver's tick kthread after a
	 * transfer completes -- set by spi_mt_transport_drv.c via
	 * mt_transport_hw_linux_set_notify().
	 */
	void (*pNotify)(void *pNotifyCtx);
	void *pNotifyCtx;
};

void mt_transport_hw_linux_init(struct mt_transport_hw_ctx *ctx, struct spi_device *spi,
				 struct gpio_desc *nss_gpiod, struct gpio_desc *nrdy_gpiod,
				 trSpiTransportHw *pHw);

void mt_transport_hw_linux_set_notify(struct mt_transport_hw_ctx *ctx,
				      void (*pNotify)(void *pNotifyCtx), void *pNotifyCtx);

/* NRDY GPIO IRQ handler (both-edges), wired by spi_mt_transport_drv.c's
 * probe() if the chosen NRDY line has usable IRQ support. Latency
 * optimization only -- tick-driven pReadyRead() polling always works too.
 */
irqreturn_t mt_transport_hw_linux_nrdy_irq(int irq, void *dev_id);

#endif /* SPI_TRANSPORT_HW_LINUX_H */
