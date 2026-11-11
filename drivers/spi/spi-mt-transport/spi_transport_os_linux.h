/* SPDX-License-Identifier: GPL-2.0 */
/*
 * spi_transport_os_linux.h - Linux kernel OS-adapter for the MultiTracks
 * SPI transport core (see ../../spi_transport_os.h for the contract this
 * implements).
 *
 * Host-role only. Maps the core's OS-adapter contract onto kernel primitives:
 * a struct completion for ISR-to-kthread handoff, a mutex for the core's
 * registration-table critical section, and a spinlock for the one
 * tick-poll-vs-IRQ race the core documents as needing IRQ-safe protection.
 */

#ifndef SPI_TRANSPORT_OS_LINUX_H
#define SPI_TRANSPORT_OS_LINUX_H

#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>

#include "spi_transport/spi_transport_os.h"

struct device;

struct mt_transport_os_ctx {
	struct device *dev;
	struct completion notify;
	struct mutex reg_lock;
	spinlock_t crit_lock;
	/* Per-CPU, not a single shared field: spin_lock_irqsave()'s saved
	 * flags must be per-caller. crit_lock can be contended from two
	 * different CPUs at once (the tick kthread vs. the NRDY IRQ path --
	 * see mt_os_critical_enter()'s comment), so a single ctx-wide field
	 * would let the losing CPU's spin overwrite the value the winning
	 * CPU needs to restore on unlock.
	 */
	unsigned long __percpu *crit_flags;
};

/*
 * Initialize ctx and fill in *pOs with function pointers bound to ctx.
 * ctx must outlive the transport instance (embed it in the driver's private
 * struct). Allocates a devm-managed per-CPU flags slot (auto-freed on
 * driver detach, same lifetime model as the driver's other devm_* state) --
 * returns 0 on success, -ENOMEM if that allocation fails.
 */
int mt_transport_os_linux_init(struct mt_transport_os_ctx *ctx, struct device *dev,
				trSpiTransportOs *pOs);

#endif /* SPI_TRANSPORT_OS_LINUX_H */
