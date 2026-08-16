//******************************************************************************
// @file      : spi_transport_os.h
// @brief     : OS-adapter contract the portable transport core depends on --
//              implemented once per platform (FreeRTOS, host-native stub,
//              future Linux kernel).
// @date      : 2026-08-10
//******************************************************************************
// @attention
//
// Copyright (c) 2026 MultiTracks.com, LLC.
// All rights reserved.
//
// For internal MultiTracks use only. Unauthorized reproduction, distribution,
// or disclosure is prohibited.
//
//******************************************************************************

#ifndef SPI_TRANSPORT_OS_H
#define SPI_TRANSPORT_OS_H

#include <stdarg.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/// @brief OS-adapter function pointers. The core never calls an RTOS/kernel
///        primitive directly -- every such call goes through this struct, so
///        the same core builds against FreeRTOS, a host-native stub, or (in
///        the future) Linux kernel primitives without core changes.
typedef struct _trSpiTransportOs
{
	/// @brief Block the calling task until the core's completion notification
	///        is given (pTaskNotifyGive) or timeoutMs elapses.
	void (*pTaskNotifyWait) (void *pContext, uint32_t timeoutMs);

	/// @brief Wake the task blocked in pTaskNotifyWait. Must be safe to call
	///        from interrupt context.
	void (*pTaskNotifyGive) (void *pContext);

	/// @brief Monotonic milliseconds since boot.
	uint32_t (*pTickGet) (void *pContext);

	/// @brief Enter/leave the core's internal critical section (registration
	///        table, per-channel TX slots). Not used from interrupt context.
	void (*pMutexLock) (void *pContext);
	void (*pMutexUnlock) (void *pContext);

	/// @brief Enter/leave a short task-vs-interrupt critical section --
	///        unlike pMutexLock/Unlock, this MUST be safe to call from (and
	///        against) interrupt context, since it guards Host's
	///        check-then-claim state transitions (hostArmTransferIfAcked,
	///        hostIssueRequestIfReady) shared between spiTransportTick()'s
	///        tick-driven poll and the NRDY-EXTI interrupt-driven path
	///        (spiTransportHwStm32OnNrdyExti). Confirmed live on hardware:
	///        without this, the two paths can race on the same
	///        check-then-act, both proceed, and Host silently double-sends
	///        (txSeq advances twice for one physical transfer), which the
	///        Client sees as a spurious sequence-gap. Keep the guarded
	///        region tiny -- a few field reads/writes only, never a HAL
	///        DMA/GPIO call that could block.
	void (*pCriticalEnter) (void *pContext);
	void (*pCriticalExit) (void *pContext);

	/// @brief Optional diagnostic log sink. May be NULL.
	void (*pLog) (void *pContext, const char *pFormat, va_list args);

	void *pContext;
} trSpiTransportOs;

#ifdef __cplusplus
}
#endif

#endif /* SPI_TRANSPORT_OS_H */
