//******************************************************************************
// @file      : spi_transport_hw.h
// @brief     : Hardware-adapter contract the portable transport core depends
//              on -- implemented once per platform (STM32 HAL SPI/DMA/GPIO,
//              host-native loopback, future Linux kernel spi_sync()).
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

#ifndef SPI_TRANSPORT_HW_H
#define SPI_TRANSPORT_HW_H

#include <stdbool.h>
#include <stdint.h>

#include "spi_transport/spi_transport_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

/// @brief Hardware-adapter function pointers. See docs/ProtocolSpec.md
///        "NSS/NRDY handshake state machine" for exactly when the core calls
///        each of these and what each signal means for Host vs. Client.
typedef struct _trSpiTransportHw
{
	/// @brief Begin a full-duplex DMA transfer of exactly
	///        SPI_TRANSPORT_FRAME_TOTAL_SIZE bytes. Must return immediately;
	///        completion is reported via the pOnTransferComplete callback
	///        (see spiTransportHwSetCallbacks below).
	///        On Client, this may now be called in two distinct contexts,
	///        both of which the adapter must handle identically: after NSS
	///        has already fallen (Host-initiated, the original contract --
	///        see pOnSelectEvent below) or *before* NSS falls, while
	///        soliciting Host (Client-initiated -- see docs/ProtocolSpec.md
	///        "Client-initiated transfers"). pAbort() below must be equally
	///        effective recovering from either case -- this is the exact
	///        contract an earlier, reverted attempt at Client-initiated
	///        transfers got wrong (no bounded recovery existed for the
	///        latter case at all, so a peer that never noticed/clocked the
	///        solicitation left the peripheral stuck indefinitely).
	teSpiTransportError (*pTransferStart) (void *pContext, const uint8_t *pTx, uint8_t *pRx,
	                                       uint16_t length);

	/// @brief Host only: drive the NSS line. Literal level convention used by
	///        every function in this struct: true = pin driven/read HIGH,
	///        false = LOW. NSS HIGH = idle, NSS LOW = request. No-op on
	///        Client.
	void (*pSelectAssert) (void *pContext, bool high);

	/// @brief Client only: drive the NRDY line. NRDY HIGH = idle/committed
	///        (rest state), NRDY LOW = ack (see docs/ProtocolSpec.md
	///        "NSS/NRDY handshake state machine"). No-op on Host.
	void (*pReadyAssert) (void *pContext, bool high);

	/// @brief Host only: read the current NRDY input level (true = HIGH =
	///        Client idle/available). Unused on Client.
	bool (*pReadyRead) (void *pContext);

	/// @brief Force the underlying transfer engine back to idle/ready when
	///        the core detects a pTransferStart() it armed never completed
	///        (peer never clocked it, or vanished mid-transfer) -- see
	///        doDisconnect() in spi_transport.c. Resetting the core's own
	///        hostState/clientState bookkeeping isn't sufficient on real
	///        hardware: a DMA transfer genuinely armed via
	///        HAL_SPI_TransmitReceive_DMA and never clocked to completion
	///        leaves the SPI peripheral itself latched busy, so every
	///        subsequent real pTransferStart() call fails forever until this
	///        is called. Must be safe to call even when nothing is armed.
	void (*pAbort) (void *pContext);

	void *pContext;

	/* Core-side callbacks, filled in by spiTransportHwSetCallbacks() below.
	 * An adapter implementation invokes these (via its own held pointer to
	 * this same trSpiTransportHw instance) when the corresponding hardware
	 * event actually happens -- e.g. from a DMA-complete ISR, the NSS-EXTI
	 * ISR, or (in the null/loopback adapter) a direct synchronous call. */
	void (*pOnTransferComplete) (void *pCoreCtx, uint16_t length);
	void (*pOnSelectEvent) (void *pCoreCtx, bool asserted);
	void (*pOnReadyEvent) (void *pCoreCtx, bool asserted);
	void (*pOnClockStart) (void *pCoreCtx);
	void *pCoreCtx;
} trSpiTransportHw;

/// @brief Register the core's callbacks with a HW adapter instance. Called
///        once during spiTransportInit(), before spiTransportStart().
/// @param pOnTransferComplete Fired when a pTransferStart() DMA transfer
///        finishes (success or hardware failure -- see length==0 convention
///        in the adapter implementation notes).
/// @param pOnSelectEvent Client only: fired on the NSS-EXTI edge (external to
///        HAL, per the board's requirement) -- asserted=true is the falling
///        edge (Host's request).
/// @param pOnReadyEvent Host only: fired on an NRDY level change, if the
///        adapter implements it via EXTI rather than pure polling. May be
///        left unused (NULL-checked by the caller) if the adapter only polls.
/// @param pOnClockStart Client only: fired the instant real clocking begins
///        (RX FIFO start indicator), distinct from and earlier than
///        pOnTransferComplete -- this is what the core uses to decide when
///        to re-assert NRDY high (the "committed, in-flight" latch), per
///        docs/ProtocolSpec.md.
void spiTransportHwSetCallbacks (trSpiTransportHw *prHw,
                                 void (*pOnTransferComplete) (void *pCoreCtx, uint16_t length),
                                 void (*pOnSelectEvent) (void *pCoreCtx, bool asserted),
                                 void (*pOnReadyEvent) (void *pCoreCtx, bool asserted),
                                 void (*pOnClockStart) (void *pCoreCtx), void *pCoreCtx);

#ifdef __cplusplus
}
#endif

#endif /* SPI_TRANSPORT_HW_H */
