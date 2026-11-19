// SPDX-License-Identifier: GPL-2.0
/*
 * spi_transport_os_linux.c - Linux kernel OS-adapter for the MultiTracks
 * SPI transport core. See spi_transport_os_linux.h.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/percpu.h>

#include "spi_transport_os_linux.h"

/*
 * pTaskNotifyWait/pTaskNotifyGive: a single-waiter completion used as a
 * repeating notify, not a one-shot. Deliberately never reinit_completion()'d:
 * struct completion's own counter already handles repeated notify/wait
 * cycles correctly on its own (a pTaskNotifyGive() that lands while not
 * waiting just leaves the counter at 1, so the next wait returns
 * immediately instead of blocking). An earlier version called
 * reinit_completion() right after the wait, which reopened a race in the
 * other direction -- a pTaskNotifyGive() landing in the (small but real)
 * window between wait_for_completion_timeout() returning and
 * reinit_completion() running would get silently discarded, delaying the
 * tick thread until the next timeout. There is exactly one waiter (the
 * driver's tick kthread), so no reinit is ever needed here.
 */
static void mt_os_task_notify_wait(void *pContext, uint32_t timeoutMs)
{
	struct mt_transport_os_ctx *ctx = pContext;

	wait_for_completion_timeout(&ctx->notify, msecs_to_jiffies(timeoutMs));
}

/* Must be IRQ-safe -- complete() is documented safe from interrupt context. */
static void mt_os_task_notify_give(void *pContext)
{
	struct mt_transport_os_ctx *ctx = pContext;

	complete(&ctx->notify);
}

static uint32_t mt_os_tick_get(void *pContext)
{
	/* Truncating a monotonic ns count to u32 ms is fine: the core only
	 * ever compares ticks via wraparound-tolerant unsigned subtraction,
	 * same as the FreeRTOS/STM32 adapter's own 32-bit millis().
	 */
	return (uint32_t)(ktime_get_ns() / NSEC_PER_MSEC);
}

/* Registration-table lock -- never taken from interrupt context. */
static void mt_os_mutex_lock(void *pContext)
{
	struct mt_transport_os_ctx *ctx = pContext;

	mutex_lock(&ctx->reg_lock);
}

static void mt_os_mutex_unlock(void *pContext)
{
	struct mt_transport_os_ctx *ctx = pContext;

	mutex_unlock(&ctx->reg_lock);
}

/*
 * Guards the one Host check-then-claim race between the tick kthread's poll
 * and the NRDY-IRQ path (see spi_transport_os.h). Must be IRQ-safe both
 * directions, and the core promises this region is only ever a few field
 * reads/writes -- never a call that could block. The core's own contract
 * never nests these calls *on a single caller's own path*, but that does
 * not mean crit_lock is never contended -- the tick kthread and the NRDY
 * IRQ handler are two different execution contexts that can genuinely run
 * on two different CPUs at once. spin_lock_irqsave()'s saved flags must
 * therefore be per-CPU (ctx->crit_flags), not a single shared field:
 * once the lock is held, preemption/local IRQs stay disabled on this CPU
 * until the matching exit, so this_cpu_ptr() is stable across the whole
 * enter/exit pair without needing get_cpu()/put_cpu().
 */
static void mt_os_critical_enter(void *pContext)
{
	struct mt_transport_os_ctx *ctx = pContext;
	unsigned long flags;

	spin_lock_irqsave(&ctx->crit_lock, flags);
	*this_cpu_ptr(ctx->crit_flags) = flags;
}

static void mt_os_critical_exit(void *pContext)
{
	struct mt_transport_os_ctx *ctx = pContext;
	unsigned long flags = *this_cpu_ptr(ctx->crit_flags);

	spin_unlock_irqrestore(&ctx->crit_lock, flags);
}

static void mt_os_log(void *pContext, const char *pFormat, va_list args)
{
	struct mt_transport_os_ctx *ctx = pContext;
	va_list args_copy;
	struct va_format vaf;

	/* On architectures where va_list is an array type, &args here would
	 * point at the local (already pointer-decayed) parameter rather than
	 * a real va_list object, breaking %pV's va_arg()-based consumption --
	 * va_copy() into a genuinely local va_list is the portable way to
	 * get something &-able regardless of the platform's va_list
	 * representation. Found by Copilot's PR #46 review.
	 */
	va_copy(args_copy, args);
	vaf.fmt = pFormat;
	vaf.va = &args_copy;

	dev_dbg(ctx->dev, "%pV", &vaf);

	va_end(args_copy);
}

int mt_transport_os_linux_init(struct mt_transport_os_ctx *ctx, struct device *dev,
				trSpiTransportOs *pOs)
{
	ctx->dev = dev;
	init_completion(&ctx->notify);
	mutex_init(&ctx->reg_lock);
	spin_lock_init(&ctx->crit_lock);

	ctx->crit_flags = devm_alloc_percpu(dev, unsigned long);
	if (!ctx->crit_flags)
		return -ENOMEM;

	pOs->pTaskNotifyWait = mt_os_task_notify_wait;
	pOs->pTaskNotifyGive = mt_os_task_notify_give;
	pOs->pTickGet = mt_os_tick_get;
	pOs->pMutexLock = mt_os_mutex_lock;
	pOs->pMutexUnlock = mt_os_mutex_unlock;
	pOs->pCriticalEnter = mt_os_critical_enter;
	pOs->pCriticalExit = mt_os_critical_exit;
	pOs->pLog = mt_os_log;
	pOs->pContext = ctx;

	return 0;
}
