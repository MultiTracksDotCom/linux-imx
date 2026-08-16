// SPDX-License-Identifier: GPL-2.0
/*
 * spi_transport_os_linux.c - Linux kernel OS-adapter for the MultiTracks
 * SPI transport core. See spi_transport_os_linux.h.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>

#include "spi_transport_os_linux.h"

/*
 * pTaskNotifyWait/pTaskNotifyGive: a single-waiter completion used as a
 * repeating notify, not a one-shot. reinit_completion() runs *after*
 * consuming the wait, not before -- reinit-before-wait would race a
 * pTaskNotifyGive() landing between the previous tick() and this wait,
 * silently swallowing the wakeup until the next timeout. There is exactly
 * one waiter (the driver's tick kthread), so this ordering is safe.
 */
static void mt_os_task_notify_wait(void *pContext, uint32_t timeoutMs)
{
	struct mt_transport_os_ctx *ctx = pContext;

	wait_for_completion_timeout(&ctx->notify, msecs_to_jiffies(timeoutMs));
	reinit_completion(&ctx->notify);
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
 * never nests these calls, so a single saved-flags field in ctx is enough;
 * this is not a general-purpose reentrant lock.
 */
static void mt_os_critical_enter(void *pContext)
{
	struct mt_transport_os_ctx *ctx = pContext;

	spin_lock_irqsave(&ctx->crit_lock, ctx->crit_flags);
}

static void mt_os_critical_exit(void *pContext)
{
	struct mt_transport_os_ctx *ctx = pContext;

	spin_unlock_irqrestore(&ctx->crit_lock, ctx->crit_flags);
}

static void mt_os_log(void *pContext, const char *pFormat, va_list args)
{
	struct mt_transport_os_ctx *ctx = pContext;
	struct va_format vaf = { .fmt = pFormat, .va = &args };

	dev_dbg(ctx->dev, "%pV", &vaf);
}

void mt_transport_os_linux_init(struct mt_transport_os_ctx *ctx, struct device *dev,
				 trSpiTransportOs *pOs)
{
	ctx->dev = dev;
	init_completion(&ctx->notify);
	mutex_init(&ctx->reg_lock);
	spin_lock_init(&ctx->crit_lock);

	pOs->pTaskNotifyWait = mt_os_task_notify_wait;
	pOs->pTaskNotifyGive = mt_os_task_notify_give;
	pOs->pTickGet = mt_os_tick_get;
	pOs->pMutexLock = mt_os_mutex_lock;
	pOs->pMutexUnlock = mt_os_mutex_unlock;
	pOs->pCriticalEnter = mt_os_critical_enter;
	pOs->pCriticalExit = mt_os_critical_exit;
	pOs->pLog = mt_os_log;
	pOs->pContext = ctx;
}
