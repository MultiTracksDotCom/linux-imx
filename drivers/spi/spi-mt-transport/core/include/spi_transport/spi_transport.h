//******************************************************************************
// @file      : spi_transport.h
// @brief     : Public API for the SPI transport -- init/start/stop, per-channel
//              registration, send, and link-wide connect/disconnect/error
//              events. See docs/ProtocolSpec.md and docs/ChannelApi.md.
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

#ifndef SPI_TRANSPORT_H
#define SPI_TRANSPORT_H

#include <stdbool.h>
#include <stdint.h>

#include "spi_transport/spi_transport_hw.h"
#include "spi_transport/spi_transport_os.h"
#include "spi_transport/spi_transport_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef void *thSpiTransport;

typedef struct _trSpiTransportConfig
{
	teSpiTransportRole role;
	const trSpiTransportOs *prOs;
	trSpiTransportHw *prHw; /* non-const: spiTransportInit() wires its own
	                           reverse callbacks into this same instance */
} trSpiTransportConfig;

/// @brief RX data callback: fired in task context (never from an ISR) when a
///        complete, reassembled message arrives on `channel`. `pBuffer` is
///        only valid for the duration of the callback.
typedef void (*tpSpiTransportRxCallback) (void *pContext, uint8_t channel, const uint8_t *pBuffer,
                                          uint16_t length, uint8_t flags);

/// @brief Link-wide event callback (connect/disconnect + transport errors,
///        see teSpiTransportEvent). Every registered channel receives the
///        same event sequence -- events are not per-channel.
typedef void (*tpSpiTransportEventCallback) (void *pContext, teSpiTransportEvent eEvent);

/// @brief One-time global init. Not thread-safe against a concurrent second
///        call; call once at startup before spiTransportStart().
teSpiTransportError spiTransportInit (const trSpiTransportConfig *prConfig,
                                      thSpiTransport *phTransport);

/// @brief Begin operation: Host starts issuing requests from IDLE; Client
///        starts waiting from IDLE. See docs/ProtocolSpec.md NSS/NRDY section.
teSpiTransportError spiTransportStart (thSpiTransport hTransport);

/// @brief Stop operation. Safe to call at any point, including mid-transfer
///        (see the fault-injection "reset during transfer" mode in
///        docs/TestPlan.md) -- must leave no stuck DMA/GPIO state behind.
void spiTransportStop (thSpiTransport hTransport);

/// @brief Register a channel for RX delivery and/or link events. Channel 0 is
///        transport-internal and always returns eSpiTransportErrorInvalidChannel.
///        pEventCallback may be NULL if this channel only cares about RX data.
teSpiTransportError spiTransportRegisterChannel (thSpiTransport hTransport, uint8_t channel,
                                                 tpSpiTransportRxCallback pRxCallback,
                                                 tpSpiTransportEventCallback pEventCallback,
                                                 void *pContext);

teSpiTransportError spiTransportDeregisterChannel (thSpiTransport hTransport, uint8_t channel);

/// @brief Queue `length` bytes for delivery on `channel`, segmenting across
///        multiple frames as needed (see SPI_TRANSPORT_FRAME_PAYLOAD_SIZE).
///        Channel 0 is transport-internal and always returns
///        eSpiTransportErrorInvalidChannel (same restriction as
///        spiTransportRegisterChannel()). Returns eSpiTransportErrorBusy if
///        the channel's previous message hasn't finished sending yet --
///        there is no hidden queue depth beyond the single in-flight
///        message per channel.
teSpiTransportError spiTransportSend (thSpiTransport hTransport, uint8_t channel,
                                      const uint8_t *pBuffer, uint16_t length, bool ackRequired);

/// @brief Current link state, for polling use (most callers should prefer the
///        event callback instead).
teSpiTransportLinkState spiTransportGetLinkState (thSpiTransport hTransport);

/// @brief Client only: enable/disable Client-initiated transfers (Client
///        arms its own DMA and drops NRDY while NSS is still high, to
///        solicit Host rather than waiting for Host's own request cycle --
///        see docs/ProtocolSpec.md "Client-initiated transfers"). No-op on
///        Host. **Defaults to disabled** at spiTransportInit() -- today's
///        validated Host-only-initiates behavior is the standing default;
///        this must be explicitly opted into. Intended to be toggled at
///        runtime (e.g. a console command mirroring the existing
///        fault-injection commands) so a hardware regression can be backed
///        out instantly without a reflash, per the staged hardware rollout
///        plan in project memory. Safe to call at any time, including
///        mid-cycle -- takes effect on the next idle opportunity to
///        self-initiate; never interrupts an already-armed cycle.
void spiTransportSetClientSelfInitEnabled (thSpiTransport hTransport, bool enabled);

/// @brief Current state of the flag set by
///        spiTransportSetClientSelfInitEnabled() -- for a console command's
///        own echo/status line, not required for the feature itself.
bool spiTransportIsClientSelfInitEnabled (thSpiTransport hTransport);

/// @brief Drive the timer-based parts of the state machine (Host's
///        effectively-every-tick heartbeat/request-issue check, both
///        roles' 1.5s disconnect check).
///        The platform integration is responsible for calling this
///        periodically (STM32: from the transport task's own loop, e.g.
///        every 1-5ms; host-native tests: called directly by the test/loopback
///        driver). Safe to call more often than needed.
void spiTransportTick (thSpiTransport hTransport);

/* Bring-up diagnostics -- temporary, see docs/TestPlan.md. Counts times the
 * deferred-RX ring (see spi_transport.c) was still full when the ISR tried
 * to write the next frame into it, i.e. the consuming task fell behind by
 * more than SPI_TRANSPORT_RX_RING_DEPTH transfers and a frame was dropped. */
uint32_t spiTransportDebugRxOverwriteCount (thSpiTransport hTransport);

/// @brief Bring-up diagnostic -- temporary. pBuiltCount: how many times
///        buildOutgoingFrame() has run (every arm attempt, success or
///        failure). pAdvanceCount: how many times txSeq actually advanced
///        (onTransferComplete confirmed a physical transfer). pCurrentTxSeq:
///        the live txSeq value. Added to directly verify on real hardware
///        that the seq-peek/commit split holds -- built and advance should
///        only ever diverge by the count of abandoned/failed arm attempts
///        (the existing dmaFail counter), never more, and never less.
void spiTransportDebugTxSeqCounts (thSpiTransport hTransport, uint32_t *pBuiltCount,
                                   uint32_t *pAdvanceCount, uint16_t *pCurrentTxSeq);

/// @brief Bring-up diagnostic -- temporary. Client only: how many times
///        onSelectEvent()/clientArmSelfInitiateIfIdle() declined to ack or
///        self-arm specifically because the deferred-RX ring
///        (SPI_TRANSPORT_RX_RING_DEPTH) had no free slot -- real
///        backpressure (Host's existing ack-wait timeout retries later)
///        rather than accepting the transfer and dropping it at the ring.
///        Climbing steadily indicates sustained, not just bursty, load.
uint32_t spiTransportDebugRxRingFullRejectedCount (thSpiTransport hTransport);

/// @brief Bring-up diagnostic -- temporary. Client only: how many real
///        NSS-falling edges onSelectEvent() has seen since boot,
///        split by outcome -- pRejectedCount (clientState wasn't Idle, so
///        the edge was ignored and NRDY was NOT touched) vs. pArmedCount
///        (clientState was Idle, pTransferStart() succeeded, and NRDY WAS
///        asserted low). Added for the DMA-wedge investigation, to
///        correlate a logic-analyzer-observed "NRDY never goes low" against
///        which path the core actually took. Either output param may be
///        NULL. Not per-instance (there's only ever one Client role active
///        per process) -- hTransport is accepted for API symmetry with the
///        other spiTransportDebugXxx calls but otherwise unused.
void spiTransportDebugClientArmCounts (thSpiTransport hTransport, uint32_t *pRejectedCount,
                                       uint32_t *pArmedCount);

/// @brief Bring-up diagnostic -- temporary. Host-side mirror of
///        spiTransportDebugClientArmCounts(): does hostArmTransferIfAcked()
///        ever actually see hostState==eHostWaitingAck with NRDY low
///        (pArmedCount, proceeds to a real pTransferStart()) vs. everything
///        else (pRejectedCount -- wrong hostState, or NRDY still high). Not
///        per-instance; hTransport accepted for API symmetry only.
void spiTransportDebugHostArmCounts (thSpiTransport hTransport, uint32_t *pRejectedCount,
                                     uint32_t *pArmedCount);

/// @brief Bring-up diagnostic -- temporary. Client-initiated-transfers
///        counters, landed alongside the feature itself (per the DMA-wedge
///        investigation's explicit lesson: visibility before the first
///        hardware test, not after). pAttemptCount: clientArmSelfInitiateIfIdle()
///        proceeded to a real pTransferStart(). pTimeoutCount: the self-arm
///        watchdog actually fired (pAbort() was called) -- climbing steadily
///        rather than rarely, especially alongside spiTransportDebugRxOverwriteCount()'s
///        sibling dmaCplt/[DIAG] counter going flat, is the direct
///        fingerprint of a stuck-peripheral regression. pHostArmedCount:
///        hostArmClientInitiatedIfIdle() succeeded (Host's side of the same
///        cycle). pRejectedSelfArmedCount: a split of
///        spiTransportDebugClientArmCounts()'s pRejectedCount, isolating
///        specifically "onSelectEvent rejected because clientState was
///        already eClientSelfArmed" (expected/healthy under self-init) from
///        everything else (still counted in that other call's
///        pRejectedCount, potentially a real problem). Any output param may
///        be NULL. Not per-instance; hTransport accepted for API symmetry
///        only.
void spiTransportDebugClientSelfInitCounts (thSpiTransport hTransport, uint32_t *pAttemptCount,
                                            uint32_t *pTimeoutCount, uint32_t *pHostArmedCount,
                                            uint32_t *pRejectedSelfArmedCount);

/// @brief Bring-up diagnostic -- temporary. Details of the most recent
///        sequence-gap event and running totals by category: exact repeat
///        of the last-seen seq (pDuplicateCount), seq skipped forward
///        (pLossCount), or anything else, e.g. behind by more than one
///        (pOtherCount). Any output param may be NULL.
void spiTransportDebugLastGap (thSpiTransport hTransport, uint16_t *pExpected, uint16_t *pActual,
                               uint32_t *pDuplicateCount, uint32_t *pLossCount,
                               uint32_t *pOtherCount);

/// @brief Bring-up diagnostic -- temporary. Our own epoch (generated once at
///        spiTransportInit()) and the peer's last-known epoch, to check
///        whether the epoch value is actually varying across reboots as
///        intended. Any output param may be NULL.
void spiTransportDebugEpoch (thSpiTransport hTransport, uint32_t *pOwnEpoch, uint32_t *pPeerEpoch,
                             bool *pPeerEpochKnown);

#ifdef __cplusplus
}
#endif

#endif /* SPI_TRANSPORT_H */
