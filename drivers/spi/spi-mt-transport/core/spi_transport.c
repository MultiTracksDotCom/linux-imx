//******************************************************************************
// @file      : spi_transport.c
// @brief     : Portable transport core -- Host/Client NSS/NRDY state machine,
//              channel-0 handshake/reconnect-baseline protocol, ~1ms
//              heartbeat / 1.5s disconnect timers. See docs/ProtocolSpec.md.
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

#include "spi_transport/spi_transport.h"

#include <string.h>

#include "spi_transport/spi_transport_channel.h"
#include "spi_transport/spi_transport_frame.h"

/* Internal watchdog: how long the Host waits for the Client's NRDY ack
 * before giving up on this one request attempt. Not part of the wire
 * protocol -- purely a local recovery bound, well under the 1.5s disconnect
 * timer so a single wedged attempt doesn't have to wait that long to retry. */
#define SPI_TRANSPORT_HOST_ACK_WAIT_TIMEOUT_MS (100u)

/* Client-side mirror of the above, for self-initiated transfers (see
 * eClientSelfArmed): how long Client will sit with its own DMA armed and
 * NRDY low, NSS still high, before concluding Host isn't going to notice
 * this solicitation. Must stay comfortably above
 * SPI_TRANSPORT_HOST_ACK_WAIT_TIMEOUT_MS + one typical transfer (so a Host
 * that's genuinely just running its own request cycle isn't mistaken for
 * "not noticing"), and comfortably below SPI_TRANSPORT_DISCONNECT_MS (this
 * is a per-attempt recovery, not a disconnect) -- see docs/ProtocolSpec.md
 * "Client-initiated transfers" for the full reasoning. */
#define SPI_TRANSPORT_CLIENT_SELF_ARM_TIMEOUT_MS (250u)

/* Depth of the deferred-RX ring (see trSpiTransportInstance's rxRingBuffer
 * comment) -- how many completed-but-not-yet-task-processed frames can
 * queue up before the ISR starts dropping new arrivals. */
#define SPI_TRANSPORT_RX_RING_DEPTH (3u)

/* Bring-up diagnostics -- temporary, see docs/TestPlan.md. Direct evidence
 * for the DMA-wedge investigation: distinguishes onSelectEvent() rejecting a
 * real NSS-falling edge because clientState wasn't Idle from it actually
 * proceeding to assert NRDY low, so a logic-analyzer-observed "NRDY never
 * goes low" can be correlated against which path the core actually took. */
volatile uint32_t gDiagOnSelectRejectedCount = 0;
volatile uint32_t gDiagOnSelectArmedCount    = 0;

/* Mirrors the pair above, Host side: does hostArmTransferIfAcked() ever
 * actually see hostState==eHostWaitingAck && NRDY low (pRejectedCount is
 * everything else -- wrong hostState, or NRDY still high) and proceed to a
 * real pTransferStart() attempt (pArmedCount)? A failed attempt is already
 * visible via the existing dmaFail/[DBG] counter, so not duplicated here. */
volatile uint32_t gDiagHostArmRejectedCount = 0;
volatile uint32_t gDiagHostArmArmedCount    = 0;

/* Client self-initiation diagnostics -- added alongside the feature itself
 * (not retrofitted after a hardware failure like the pairs above), per the
 * explicit lesson from the DMA-wedge investigation: land the visibility
 * BEFORE the first hardware test, not after. gDiagClientSelfArmTimeoutCount
 * climbing steadily (not just occasionally) alongside the existing
 * gDiagDmaCompleteCount/dmaCplt counter going flat is the direct
 * fingerprint of a stuck-peripheral regression -- visible within one
 * [DIAG] print interval instead of requiring a fresh logic-analyzer
 * capture to diagnose. */
volatile uint32_t gDiagClientSelfArmAttemptCount = 0; /* clientArmSelfInitiateIfIdle attempted */
volatile uint32_t gDiagClientSelfArmTimeoutCount = 0; /* watchdog actually fired (pAbort called) */
volatile uint32_t gDiagHostClientInitArmedCount  = 0; /* hostArmClientInitiatedIfIdle succeeded */
volatile uint32_t gDiagOnSelectRejectedSelfArmedCount = 0; /* split of
    gDiagOnSelectRejectedCount: specifically "rejected because clientState
    was eClientSelfArmed" (expected/healthy under self-init) vs. anything
    else (still counted in gDiagOnSelectRejectedCount, potentially a real
    problem). */

/* TX-side seq-consumption diagnostics -- added to directly verify the
 * buildOutgoingFrame()/onTransferComplete() seq-peek/commit split (see
 * buildOutgoingFrame's own comment) actually holds on real hardware, not
 * just in native tests: gDiagTxFrameBuiltCount increments once per
 * buildOutgoingFrame() call (every arm attempt, success or failure);
 * gDiagTxSeqAdvanceCount increments once per confirmed txSeq++ in
 * onTransferComplete(). The two are expected to diverge by exactly the
 * count of abandoned/failed arm attempts (dmaFail) -- built-advance should
 * never exceed that, and advance should never exceed built. See
 * spiTransportDebugTxSeqCounts(). */
volatile uint32_t gDiagTxFrameBuiltCount = 0;
volatile uint32_t gDiagTxSeqAdvanceCount = 0;

/* RX-ring backpressure diagnostic -- Client only. Counts how many times
 * onSelectEvent()/clientArmSelfInitiateIfIdle() declined to ack/self-arm
 * specifically because the deferred-RX ring had no free slot (see
 * SPI_TRANSPORT_RX_RING_DEPTH), as opposed to being busy for some other
 * reason. Climbing steadily means the ring is genuinely undersized for the
 * sustained load, not just absorbing rare bursts -- see
 * spiTransportDebugRxRingFullRejectedCount(). */
volatile uint32_t gDiagRxRingFullRejectedCount = 0;

/* Control-channel (channel 0) message body: {type, role, epoch(LE32), startSeq(LE16)}. */
#define SPI_TRANSPORT_CTRL_BODY_SIZE (8u)

typedef enum
{
	eHostIdle = 0,
	eHostWaitingAck,
	eHostTransferring,
} teHostState;

typedef enum
{
	eClientIdle = 0,
	/* Self-initiated: Client armed its own DMA and dropped NRDY while NSS is
	 * still HIGH, soliciting Host -- distinct from eClientArmed (which is
	 * always a reaction to Host's own NSS-falling edge). Bounded by
	 * SPI_TRANSPORT_CLIENT_SELF_ARM_TIMEOUT_MS (see clientServiceTick());
	 * onClockStart() moves this on to eClientTransferring exactly like
	 * eClientArmed, since once real clocking starts it no longer matters
	 * which side asked for it. */
	eClientSelfArmed,
	eClientArmed,
	eClientTransferring,
} teClientState;

typedef struct _trSpiTransportInstance
{
	bool inUse;
	teSpiTransportRole role;
	const trSpiTransportOs *prOs;
	trSpiTransportHw *prHw;

	trSpiTransportChannelTable channels;

	teSpiTransportLinkState linkState;
	uint32_t epoch;
	uint32_t peerEpoch;
	bool peerEpochKnown;
	bool helloSentThisEpoch;
	bool pendingHello;
	bool pendingHelloAck;
	uint16_t startSeqFromPeer;

	uint16_t txSeq;
	uint16_t rxLastSeq;
	bool rxSeqBaselineArmed;

	uint8_t lastTxChannel;

	uint32_t lastSendTickMs;
	uint32_t lastRecvTickMs;

	teHostState hostState;
	uint32_t hostWaitStartMs;

	teClientState clientState;
	uint32_t clientSelfArmStartMs; /* set when clientState becomes eClientSelfArmed */
	bool clientSelfInitEnabled; /* see spiTransportSetClientSelfInitEnabled() -- defaults false */

	/* Deferred TX-commit bookkeeping: buildOutgoingFrame() only PEEKS a
	 * channel's next chunk (see spiTransportChannelNextTx()'s updated
	 * contract) -- these record which chunk was peeked into the
	 * currently-in-flight frame, so the transfer-complete confirmation
	 * (onTransferComplete) can commit it, or a failed/abandoned attempt can
	 * simply leave it uncommitted for a clean retry. */
	bool txCommitPending;
	uint8_t txCommitChannel;
	uint16_t txCommitChunkLen;
	bool txCommitWasLastChunk;

	uint8_t txBuffer[SPI_TRANSPORT_FRAME_TOTAL_SIZE];
	uint8_t rxBuffer[SPI_TRANSPORT_FRAME_TOTAL_SIZE];

	/* Deferred-processing handoff: the ISR-context transfer-complete handler
	 * (see onTransferComplete) only snapshots rxBuffer here and notifies the
	 * task -- it must NOT call processIncomingFrame() itself, since that
	 * invokes user RX/event callbacks, which docs/ProtocolSpec.md requires
	 * to run in task context only. spiTransportTick() drains this.
	 *
	 * Single-producer (ISR)/single-consumer (task) ring, depth
	 * SPI_TRANSPORT_RX_RING_DEPTH: rxRingHead is written only by the
	 * producer, rxRingTail only by the consumer, so each side can read the
	 * other's counter without a lock. Index into rxRingBuffer is
	 * (counter % SPI_TRANSPORT_RX_RING_DEPTH); both counters are
	 * free-running (never reduced mod the depth themselves), so unsigned
	 * wraparound subtraction (head - tail) always gives the correct
	 * occupied-slot count. Replaces an earlier single-slot design (a
	 * depth-2 ring was tried once before to close this same starved-task
	 * window, reordered frames on real hardware for a reason never
	 * isolated, and was reverted back to the single slot rather than ship
	 * that). This version avoids the most likely cause of that class of
	 * bug by construction: on a full ring, the ISR drops the *new* arrival
	 * (see the ring-full branch in onTransferComplete) rather than
	 * overwriting the oldest slot in place, which would race the task
	 * mid-read of that exact slot -- the ISR can preempt the task at any
	 * point, so slot ownership must never be ambiguous. See
	 * test/host/test_rx_ring.c for the back-to-back-completion/FIFO-order
	 * coverage this earlier attempt apparently didn't have. */
	uint8_t rxRingBuffer[SPI_TRANSPORT_RX_RING_DEPTH][SPI_TRANSPORT_FRAME_TOTAL_SIZE];
	volatile uint32_t rxRingHead;
	volatile uint32_t rxRingTail;

	/* Bring-up diagnostic -- temporary, see spiTransportDebugRxOverwriteCount().
	 * Now counts "ring was full, newest arrival dropped" rather than
	 * "existing slot overwritten", but is exactly as rare/meaningful a
	 * backpressure signal as before. */
	volatile uint32_t rxOverwriteCount;

	/* Bring-up diagnostic -- temporary, see spiTransportDebugLastGap(). Only
	 * ever written from task context (processIncomingFrame), so no
	 * volatile/critical-section needed here. */
	uint16_t lastGapExpected;
	uint16_t lastGapActual;
	uint32_t gapDuplicateCount; /* frame.seq == rxLastSeq (exact repeat) */
	uint32_t gapLossCount;      /* frame.seq ahead of expected (skipped) */
	uint32_t gapOtherCount;     /* anything else (behind by >1, wrap, etc.) */
} trSpiTransportInstance;

/* Static pool, no malloc anywhere (see docs/ProtocolSpec.md "Channel
 * model"). Real firmware only ever uses one slot (one physical link per
 * board); a size of 2 is what lets host-native tests run a Host instance
 * and a Client instance simultaneously in the same process against the
 * null-loopback HW adapter (see platform/host/spi_transport_hw_null.c). */
#define SPI_TRANSPORT_INSTANCES_MAX (2u)
static trSpiTransportInstance gInstances[SPI_TRANSPORT_INSTANCES_MAX];

static void onTransferComplete (void *pCoreCtx, uint16_t length);
static void onSelectEvent (void *pCoreCtx, bool asserted);
static void onReadyEvent (void *pCoreCtx, bool asserted);
static void onClockStart (void *pCoreCtx);

static void
putU16 (uint8_t *pOut, uint16_t value)
{
	pOut[0] = (uint8_t)(value & 0xFFu);
	pOut[1] = (uint8_t)((value >> 8) & 0xFFu);
}
static uint16_t
getU16 (const uint8_t *pIn)
{
	return (uint16_t)((uint16_t)pIn[0] | ((uint16_t)pIn[1] << 8));
}
static void
putU32 (uint8_t *pOut, uint32_t value)
{
	pOut[0] = (uint8_t)(value & 0xFFu);
	pOut[1] = (uint8_t)((value >> 8) & 0xFFu);
	pOut[2] = (uint8_t)((value >> 16) & 0xFFu);
	pOut[3] = (uint8_t)((value >> 24) & 0xFFu);
}
static uint32_t
getU32 (const uint8_t *pIn)
{
	return (uint32_t)pIn[0] | ((uint32_t)pIn[1] << 8) | ((uint32_t)pIn[2] << 16)
	       | ((uint32_t)pIn[3] << 24);
}

static void
updateLinkState (trSpiTransportInstance *pInst)
{
	bool shouldBeConnected = pInst->peerEpochKnown && pInst->helloSentThisEpoch;

	if (shouldBeConnected && (pInst->linkState != eSpiTransportLinkConnected))
		{
			pInst->linkState = eSpiTransportLinkConnected;
			spiTransportChannelNotifyEvent (&pInst->channels, eSpiTransportEventConnected);
		}
}

static void
resetForHandshake (trSpiTransportInstance *pInst)
{
	pInst->linkState          = eSpiTransportLinkHandshaking;
	pInst->helloSentThisEpoch = false;
	pInst->pendingHello       = true;
	pInst->rxSeqBaselineArmed = false;
	spiTransportChannelResetAll (&pInst->channels);

	/* Any commit still deferred against the (now-wiped) channel table would
	 * be meaningless -- drop it rather than risk a later onTransferComplete
	 * committing stale channel/offset values into a freshly reset slot. */
	pInst->txCommitPending = false;
}

static void
handleHandshake (trSpiTransportInstance *pInst, uint8_t msgType, uint32_t peerEpoch,
                 uint16_t peerStartSeq)
{
	bool epochChanged = (!pInst->peerEpochKnown) || (peerEpoch != pInst->peerEpoch);

	/* A peer only ever sends a fresh HELLO (as opposed to a HELLO_ACK)
	 * right after its own Start()/reboot or its own detected disconnect --
	 * see buildOutgoingFrame's pendingHello handling, which is never
	 * re-armed spontaneously mid-connection. So an incoming HELLO while we
	 * still think we're Connected is itself sufficient proof the peer
	 * restarted, and must be treated as a reconnect even when epochChanged
	 * is false. Confirmed live on hardware that epoch alone is NOT reliable
	 * enough on its own to catch this: epoch is a boot-time RTOS tick
	 * snapshot taken in spiTransportInit(), and on this harness's Client
	 * role that call consistently lands before the scheduler's first tick
	 * -- every single Client reboot produces epoch=0, a guaranteed
	 * collision with whatever Host already had stored, not a rare
	 * probabilistic one. Without this OR clause, Host silently never
	 * re-baselines against a rebooted Client: no Disconnected event, no
	 * rxSeqBaselineArmed reset (each such miss produces one real spurious
	 * sequence-gap event where the peer's seq legitimately restarted from
	 * ~0), and recovery depends entirely on the far coarser, timing-
	 * dependent 1.5s silence timeout instead. */
	bool peerRestarted = epochChanged
	                     || ((msgType == SPI_TRANSPORT_CTRL_HELLO)
	                         && (pInst->linkState == eSpiTransportLinkConnected));

	if (peerRestarted)
		{
			bool wasConnected = (pInst->linkState == eSpiTransportLinkConnected);

			pInst->peerEpoch        = peerEpoch;
			pInst->peerEpochKnown   = true;
			pInst->startSeqFromPeer = peerStartSeq;
			resetForHandshake (pInst);

			if (wasConnected)
				{
					spiTransportChannelNotifyEvent (&pInst->channels,
					                                eSpiTransportEventDisconnected);
				}
		}

	if (msgType == SPI_TRANSPORT_CTRL_HELLO)
		{
			pInst->pendingHelloAck = true;
		}

	updateLinkState (pInst);
}

static void
doDisconnect (trSpiTransportInstance *pInst, uint32_t now)
{
	bool wasConnected = (pInst->linkState == eSpiTransportLinkConnected);

	pInst->peerEpochKnown = false;
	pInst->epoch          = pInst->prOs->pTickGet (pInst->prOs->pContext);
	resetForHandshake (pInst);
	pInst->lastRecvTickMs = now;

	/* Un-wedge the NSS/NRDY handshake, not just the protocol state above:
	 * confirmed live on hardware that if the peer vanishes (e.g. reboots)
	 * mid-transfer, the surviving side's hostState/clientState is left
	 * stuck in a Waiting/Armed/Transferring state forever -- its DMA is
	 * waiting on clock edges or an ack that will never arrive, and nothing
	 * else ever moves it back to Idle. That left NRDY (Client) or NSS
	 * (Host) permanently latched, wedging the physical link even though
	 * the 1.5s disconnect timer above correctly recovered the protocol
	 * layer. Force both back to Idle and release the line here as the
	 * disconnect timer's backstop -- Host also has its own tighter 100ms
	 * per-attempt watchdog (see hostServiceTick), this is what actually
	 * covers Client, which has no per-attempt watchdog of its own. */
	if (pInst->role == eSpiTransportRoleHost)
		{
			/* Only eHostTransferring means a physical transfer was actually
			 * armed and never confirmed complete -- eHostWaitingAck gets
			 * its own 100ms watchdog well before this 1.5s path could ever
			 * see it stuck. That distinction is what makes this a DMA
			 * timeout specifically, not just "the link is down" (which
			 * eSpiTransportEventDisconnected below already covers). */
			bool wasArmed = (pInst->hostState == eHostTransferring);
			if (wasArmed)
				{
					/* Abort the real transfer BEFORE flipping hostState back
					 * to Idle below -- hostIssueRequestIfReady/
					 * hostArmTransferIfAcked gate a new real
					 * pTransferStart() on hostState, so as long as it still
					 * reads non-Idle here, a concurrent tick/EXTI can't slip
					 * a new attempt in against a peripheral that's still
					 * mid-abort. Confirmed live on hardware that resetting
					 * hostState first (the original order) opens exactly
					 * that window -- the SPI peripheral's own HAL state
					 * (HAL_SPI_STATE_BUSY_TX_RX) is what's actually latched,
					 * not just this instance's bookkeeping, so a transfer
					 * armed on top of an in-progress abort just fails again
					 * with HAL_BUSY forever. */
					pInst->prHw->pAbort (pInst->prHw->pContext);
				}
			if (pInst->hostState != eHostIdle)
				{
					pInst->hostState = eHostIdle;
					pInst->prHw->pSelectAssert (pInst->prHw->pContext, true);
				}
			if (wasArmed)
				{
					spiTransportChannelNotifyEvent (&pInst->channels,
					                                eSpiTransportEventErrorDmaTimeout);
				}
		}
	else
		{
			bool wasArmed = (pInst->clientState != eClientIdle);
			if (wasArmed)
				{
					/* See the matching Host-side comment above -- same
					 * ordering requirement: onSelectEvent's own guard
					 * (clientState != eClientIdle) is what keeps a real
					 * NSS-falling edge from re-arming the Client's slave DMA
					 * while this abort is still in flight, so it must run
					 * before clientState is reset below. */
					pInst->prHw->pAbort (pInst->prHw->pContext);
				}
			if (pInst->clientState != eClientIdle)
				{
					pInst->clientState = eClientIdle;
					pInst->prHw->pReadyAssert (pInst->prHw->pContext, true);
				}
			if (wasArmed)
				{
					spiTransportChannelNotifyEvent (&pInst->channels,
					                                eSpiTransportEventErrorDmaTimeout);
				}
		}

	if (wasConnected)
		{
			spiTransportChannelNotifyEvent (&pInst->channels, eSpiTransportEventDisconnected);
		}
}

static uint16_t
buildControlBody (uint8_t *pOut, uint8_t type, teSpiTransportRole role, uint32_t epoch,
                  uint16_t startSeq)
{
	pOut[0] = type;
	pOut[1] = (uint8_t)role;
	putU32 (&pOut[2], epoch);
	putU16 (&pOut[6], startSeq);
	return SPI_TRANSPORT_CTRL_BODY_SIZE;
}

/// @brief Choose and encode the next outbound frame: pending HELLO/HELLO_ACK
///        first, else a channel with queued data (round-robin), else a
///        FILLER -- see docs/ProtocolSpec.md "Channel-0 handshake" and
///        "Connected/disconnected timing model".
static void
buildOutgoingFrame (trSpiTransportInstance *pInst, uint8_t *pOutBuffer)
{
	uint8_t payload[SPI_TRANSPORT_FRAME_PAYLOAD_SIZE];
	uint8_t channel = SPI_TRANSPORT_CHANNEL_CONTROL;
	uint8_t flags   = 0;
	uint16_t length = 0;

	gDiagTxFrameBuiltCount++;
	pInst->txCommitPending = false;

	if (pInst->pendingHello)
		{
			length = buildControlBody (payload, SPI_TRANSPORT_CTRL_HELLO, pInst->role, pInst->epoch,
			                           pInst->txSeq);
			flags  = SPI_TRANSPORT_FLAG_START | SPI_TRANSPORT_FLAG_END | SPI_TRANSPORT_FLAG_RESET;
			pInst->pendingHello       = false;
			pInst->helloSentThisEpoch = true;
		}
	else if (pInst->pendingHelloAck)
		{
			length = buildControlBody (payload, SPI_TRANSPORT_CTRL_HELLO_ACK, pInst->role,
			                           pInst->epoch, pInst->txSeq);
			flags  = SPI_TRANSPORT_FLAG_START | SPI_TRANSPORT_FLAG_END;
			if (pInst->linkState != eSpiTransportLinkConnected)
				{
					flags = (uint8_t)(flags | SPI_TRANSPORT_FLAG_RESET);
				}
			pInst->pendingHelloAck = false;
		}
	else
		{
			uint8_t dataChannel;
			uint16_t dataLength;
			uint8_t dataFlags;

			if (spiTransportChannelNextTx (&pInst->channels, pInst->lastTxChannel, &dataChannel,
			                               payload, &dataLength, &dataFlags))
				{
					channel              = dataChannel;
					length               = dataLength;
					flags                = dataFlags;
					pInst->lastTxChannel = dataChannel;

					/* Peeked only -- not committed until the physical
					 * transfer carrying this frame is confirmed complete
					 * (see onTransferComplete). Left uncommitted, an
					 * abandoned/failed attempt naturally retries this exact
					 * chunk next time, since spiTransportChannelNextTx()
					 * didn't advance anything either. */
					pInst->txCommitPending      = true;
					pInst->txCommitChannel      = dataChannel;
					pInst->txCommitChunkLen     = dataLength;
					pInst->txCommitWasLastChunk = (dataFlags & SPI_TRANSPORT_FLAG_END) != 0;
				}
			else
				{
					flags = SPI_TRANSPORT_FLAG_FILLER;
					if (pInst->linkState != eSpiTransportLinkConnected)
						{
							flags = (uint8_t)(flags | SPI_TRANSPORT_FLAG_RESET);
						}
				}
		}

	trSpiTransportFrame frame;
	frame.version       = SPI_TRANSPORT_FRAME_VERSION;
	frame.channel       = channel;
	frame.seq           = pInst->txSeq;
	frame.ack           = pInst->rxLastSeq;
	frame.flags         = flags;
	frame.payloadLength = length;

	(void)spiTransportFrameEncode (&frame, payload, pOutBuffer);

	/* txSeq is peeked here, not consumed -- mirrors the channel-data
	 * peek/commit split above. It only advances in onTransferComplete(),
	 * once the physical transfer this frame belongs to is confirmed to
	 * have actually gone out. An attempt that never completes (DMA-arm
	 * failure, watchdog-timeout abort, disconnect mid-transfer) leaves
	 * txSeq unchanged, so the next call re-peeks and re-sends this exact
	 * seq value instead of skipping it -- skipping it here previously
	 * meant the peer never saw that seq number at all, surfacing as a
	 * phantom sequence-gap error on every failed-then-retried attempt even
	 * though the actual channel data was correctly retried underneath. */

	pInst->lastSendTickMs = pInst->prOs->pTickGet (pInst->prOs->pContext);
}

/// @brief Decode and process one inbound frame -- handshake/epoch tracking,
///        sequence-gap detection, and channel dispatch.
static void
processIncomingFrame (trSpiTransportInstance *pInst, const uint8_t *pInBuffer)
{
	trSpiTransportFrame frame;
	teSpiTransportError err = spiTransportFrameDecode (pInBuffer, &frame);

	if (err == eSpiTransportErrorHardwareFailure)
		{
			if (!spiTransportFrameHeaderCrcOk (pInBuffer))
				{
					spiTransportChannelNotifyEvent (&pInst->channels,
					                                eSpiTransportEventErrorHeaderCrc);
				}
			else
				{
					spiTransportChannelNotifyEvent (&pInst->channels,
					                                eSpiTransportEventErrorPayloadCrc);
				}
			return;
		}
	if (err != eSpiTransportErrorNone)
		{
			return; /* bad magic / malformed -- dropped silently */
		}

	pInst->lastRecvTickMs = pInst->prOs->pTickGet (pInst->prOs->pContext);

	if ((frame.channel == SPI_TRANSPORT_CHANNEL_CONTROL)
	    && (frame.payloadLength >= SPI_TRANSPORT_CTRL_BODY_SIZE))
		{
			uint8_t msgType = frame.pPayload[0];
			if ((msgType == SPI_TRANSPORT_CTRL_HELLO) || (msgType == SPI_TRANSPORT_CTRL_HELLO_ACK))
				{
					uint32_t peerEpoch    = getU32 (&frame.pPayload[2]);
					uint16_t peerStartSeq = getU16 (&frame.pPayload[6]);
					handleHandshake (pInst, msgType, peerEpoch, peerStartSeq);
				}
		}

	if (pInst->linkState == eSpiTransportLinkConnected)
		{
			if (pInst->rxSeqBaselineArmed)
				{
					uint16_t expected = (uint16_t)(pInst->rxLastSeq + 1u);
					if (frame.seq != expected)
						{
							pInst->lastGapExpected = expected;
							pInst->lastGapActual   = frame.seq;
							if (frame.seq == pInst->rxLastSeq)
								{
									pInst->gapDuplicateCount++;
								}
							else if ((uint16_t)(frame.seq - expected) < 0x8000u)
								{
									pInst->gapLossCount++;
								}
							else
								{
									pInst->gapOtherCount++;
								}
							spiTransportChannelNotifyEvent (&pInst->channels,
							                                eSpiTransportEventErrorSequenceGap);
						}
				}
			else
				{
					pInst->rxSeqBaselineArmed = true;
				}
			pInst->rxLastSeq = frame.seq;

			if (frame.channel != SPI_TRANSPORT_CHANNEL_CONTROL)
				{
					spiTransportChannelDispatchRx (&pInst->channels, frame.channel, frame.pPayload,
					                               frame.payloadLength, frame.flags);
				}
		}
}

/* Host only. If waiting on the Client's ack and NRDY currently reads low,
 * start the real clocked transfer. Shared between the tick-driven poll
 * (hostServiceTick) and the interrupt-driven path (onReadyEvent) -- see
 * docs/ProtocolSpec.md section 6; the tick poll remains correct on its own, the
 * EXTI path just cuts the latency of noticing the edge.
 *
 * The check-then-claim on hostState is wrapped in pCriticalEnter/Exit
 * because both callers (tick-driven and EXTI-driven) can genuinely race:
 * confirmed live on hardware that without this guard, the task can read
 * hostState==eHostWaitingAck, get preempted by the NRDY-falling EXTI before
 * writing eHostTransferring, and the ISR's own call also passes the same
 * (still-stale) check -- both then call buildOutgoingFrame(), advancing
 * txSeq twice for what becomes only one physical transfer, which the Client
 * observes as a spurious sequence-gap. */
static void
hostArmTransferIfAcked (trSpiTransportInstance *pInst)
{
	pInst->prOs->pCriticalEnter (pInst->prOs->pContext);
	bool shouldArm = (pInst->hostState == eHostWaitingAck)
	                 && (pInst->prHw->pReadyRead (pInst->prHw->pContext) == false);
	if (shouldArm)
		{
			pInst->hostState = eHostTransferring;
		}
	pInst->prOs->pCriticalExit (pInst->prOs->pContext);

	if (!shouldArm)
		{
			gDiagHostArmRejectedCount++;
			return;
		}
	gDiagHostArmArmedCount++;
	buildOutgoingFrame (pInst, pInst->txBuffer);
	teSpiTransportError startErr = pInst->prHw->pTransferStart (
	    pInst->prHw->pContext, pInst->txBuffer, pInst->rxBuffer, SPI_TRANSPORT_FRAME_TOTAL_SIZE);

	if (startErr != eSpiTransportErrorNone)
		{
			/* DMA never actually armed -- nothing was clocked out, so
			 * onTransferComplete will never fire for this attempt. Abandon
			 * the peeked chunk (leave it uncommitted -- the same channel
			 * offset/txPending state as before this call, so the next
			 * successful attempt naturally retries it) and release NSS
			 * immediately rather than waiting out the 100ms ack-wait
			 * timeout or the 1.5s disconnect backstop for something we
			 * already know failed right now. */
			pInst->txCommitPending = false;
			pInst->hostState       = eHostIdle;
			pInst->prHw->pSelectAssert (pInst->prHw->pContext, true);
			spiTransportChannelNotifyEvent (&pInst->channels, eSpiTransportEventErrorDmaFailure);
		}
}

/* Host only. Disambiguates a NRDY-falling edge/observation seen while
 * hostState==eHostIdle: Host never drives NRDY itself and Client only ever
 * lowers it either responding to Host's own request (which implies
 * hostState==eHostWaitingAck, handled by hostArmTransferIfAcked() above, not
 * this function) or self-initiating (clientState==eClientSelfArmed, see
 * clientArmSelfInitiateIfIdle()). So hostState==eHostIdle observing NRDY low
 * can only mean the latter -- no new wire signal needed, Host's own two-state
 * distinction already disambiguates this for free. Skips straight to
 * eHostTransferring (no separate ack step -- the ack already happened, it's
 * what triggered this call). Same check-then-claim-under-critical-section
 * shape as hostArmTransferIfAcked(), for the same double-arm-race reason. */
static void
hostArmClientInitiatedIfIdle (trSpiTransportInstance *pInst)
{
	pInst->prOs->pCriticalEnter (pInst->prOs->pContext);
	bool shouldArm = (pInst->hostState == eHostIdle)
	                 && (pInst->prHw->pReadyRead (pInst->prHw->pContext) == false);
	if (shouldArm)
		{
			pInst->hostState = eHostTransferring;
		}
	pInst->prOs->pCriticalExit (pInst->prOs->pContext);

	if (!shouldArm)
		{
			return;
		}
	gDiagHostClientInitArmedCount++;
	/* NSS low: lets the hardware-NSS-managed Client peripheral (already
	 * self-armed, waiting) actually begin shifting, and gives Client's own
	 * NSS-EXTI a real edge -- onSelectEvent's existing clientState!=eClientIdle
	 * guard correctly ignores it there (Client is eClientSelfArmed, not
	 * Idle), so no change needed on that side. */
	pInst->prHw->pSelectAssert (pInst->prHw->pContext, false);
	buildOutgoingFrame (pInst, pInst->txBuffer);
	teSpiTransportError startErr = pInst->prHw->pTransferStart (
	    pInst->prHw->pContext, pInst->txBuffer, pInst->rxBuffer, SPI_TRANSPORT_FRAME_TOTAL_SIZE);

	if (startErr != eSpiTransportErrorNone)
		{
			pInst->txCommitPending = false;
			pInst->hostState       = eHostIdle;
			pInst->prHw->pSelectAssert (pInst->prHw->pContext, true);
			spiTransportChannelNotifyEvent (&pInst->channels, eSpiTransportEventErrorDmaFailure);
		}
}

/* Host only. If idle and something is queued/due, and NRDY currently reads
 * high (Client available), issue the next request. Shared the same way as
 * hostArmTransferIfAcked() above -- same race, same critical-section fix. */
static void
hostIssueRequestIfReady (trSpiTransportInstance *pInst, uint32_t now)
{
	bool haveSomethingQueued = pInst->pendingHello || pInst->pendingHelloAck
	                           || spiTransportChannelHasPending (&pInst->channels);
	bool heartbeatDue        = (now - pInst->lastSendTickMs) >= SPI_TRANSPORT_HEARTBEAT_MS;

	pInst->prOs->pCriticalEnter (pInst->prOs->pContext);
	bool shouldIssue = (pInst->hostState == eHostIdle) && (haveSomethingQueued || heartbeatDue)
	                   && (pInst->prHw->pReadyRead (pInst->prHw->pContext) == true);
	if (shouldIssue)
		{
			pInst->hostState       = eHostWaitingAck;
			pInst->hostWaitStartMs = now;
		}
	pInst->prOs->pCriticalExit (pInst->prOs->pContext);

	if (!shouldIssue)
		{
			return; /* not idle, nothing to send, or Client not currently available */
		}

	pInst->prHw->pSelectAssert (pInst->prHw->pContext, false); /* LOW = issue the request */
}

static void
hostServiceTick (trSpiTransportInstance *pInst, uint32_t now)
{
	if (pInst->hostState == eHostWaitingAck)
		{
			hostArmTransferIfAcked (pInst);
			if ((pInst->hostState == eHostWaitingAck)
			    && ((now - pInst->hostWaitStartMs) >= SPI_TRANSPORT_HOST_ACK_WAIT_TIMEOUT_MS))
				{
					/* Client never acked this attempt -- give up on it, let the next
					 * heartbeat/data-queued opportunity retry (the 1.5s disconnect
					 * timer is the backstop if the link is genuinely down). */
					pInst->prHw->pSelectAssert (pInst->prHw->pContext,
					                            true); /* HIGH = idle, give up this attempt */
					pInst->hostState = eHostIdle;
				}
			return;
		}

	if (pInst->hostState != eHostIdle)
		{
			return; /* eHostTransferring: waiting on onTransferComplete */
		}

	/* Poll-path fallback for platforms without a NRDY EXTI (onReadyEvent's
	 * falling-edge branch is the latency-optimized path for those that have
	 * one): a Client self-initiation could have dropped NRDY between ticks
	 * with nothing to interrupt on. Re-check hostState afterward -- a
	 * successful claim here moves it to eHostTransferring. */
	hostArmClientInitiatedIfIdle (pInst);
	if (pInst->hostState != eHostIdle)
		{
			return;
		}

	hostIssueRequestIfReady (pInst, now);
}

static void
onTransferComplete (void *pCoreCtx, uint16_t length)
{
	trSpiTransportInstance *pInst = (trSpiTransportInstance *)pCoreCtx;
	(void)length;

	if (pInst->role == eSpiTransportRoleHost)
		{
			pInst->prHw->pSelectAssert (pInst->prHw->pContext,
			                            true); /* HIGH = deassert, transfer done */
			pInst->hostState = eHostIdle;
		}
	else
		{
			pInst->clientState = eClientIdle;
		}

	/* This callback firing at all is the confirmation the just-armed
	 * transfer's outgoing bytes were actually clocked out -- only now is
	 * it safe to commit whatever channel chunk buildOutgoingFrame() peeked
	 * into this frame, and to consume the seq value it peeked alongside
	 * it (see buildOutgoingFrame's comment). Every completed transfer
	 * consumes exactly one seq value, regardless of whether it carried
	 * real channel data or was a HELLO/FILLER frame. */
	pInst->txSeq++;
	gDiagTxSeqAdvanceCount++;
	if (pInst->txCommitPending)
		{
			spiTransportChannelCommitTx (&pInst->channels, pInst->txCommitChannel,
			                             pInst->txCommitChunkLen, pInst->txCommitWasLastChunk);
			pInst->txCommitPending = false;
		}

	/* ISR context on real hardware -- must not call processIncomingFrame()
	 * here (it invokes user callbacks). Snapshot into the ring and defer to
	 * spiTransportTick(), which runs in task context. Producer side only:
	 * reads rxRingTail (written only by the consumer) to compute occupancy,
	 * writes rxRingHead. If the ring is full, drop this newest arrival
	 * (rare in practice, see spiTransportDebugRxOverwriteCount) rather than
	 * evicting an existing slot the task might be mid-read of -- this ISR
	 * can preempt the task at any point, so an in-place slot overwrite here
	 * would race that read. */
	uint32_t head = pInst->rxRingHead;
	uint32_t tail = pInst->rxRingTail;
	if ((head - tail) >= SPI_TRANSPORT_RX_RING_DEPTH)
		{
			pInst->rxOverwriteCount++;
		}
	else
		{
			memcpy (pInst->rxRingBuffer[head % SPI_TRANSPORT_RX_RING_DEPTH], pInst->rxBuffer,
			        SPI_TRANSPORT_FRAME_TOTAL_SIZE);
			pInst->rxRingHead = head + 1u;
			pInst->prOs->pTaskNotifyGive (pInst->prOs->pContext);
		}
}

static void
onSelectEvent (void *pCoreCtx, bool asserted)
{
	trSpiTransportInstance *pInst = (trSpiTransportInstance *)pCoreCtx;

	/* Backpressure: if the deferred-RX ring has no free slot, do NOT ack
	 * this request at all -- leave NRDY high, exactly as if Client weren't
	 * ready. Host's existing 100ms ack-wait timeout (hostServiceTick) is
	 * already the correct recovery path for "Client didn't ack this
	 * attempt", so this just reuses it instead of acking a transfer whose
	 * received frame would only get dropped at the ring anyway (see
	 * onTransferComplete's ring-full branch). Deferring the attempt this
	 * way, rather than accepting and silently losing it, is the actual fix
	 * for the RX-ring-overflow investigation -- not a replacement for it,
	 * a second layer: the loop-timing fix (see harnessMain) closes the
	 * dominant cause (blocking UART stalls), this closes the residual gap
	 * for whatever legitimate backlog remains. Checked before the
	 * clientState guard below since it's a distinct, worth-tracking-
	 * separately reason to decline. */
	bool ringFull = (pInst->rxRingHead - pInst->rxRingTail) >= SPI_TRANSPORT_RX_RING_DEPTH;

	if ((pInst->role != eSpiTransportRoleClient) || !asserted || (pInst->clientState != eClientIdle)
	    || ringFull)
		{
			if ((pInst->role == eSpiTransportRoleClient) && asserted)
				{
					if (ringFull && (pInst->clientState == eClientIdle))
						{
							gDiagRxRingFullRejectedCount++;
							return;
						}
					gDiagOnSelectRejectedCount++;
					/* Real NSS falling while already eClientSelfArmed is the
					 * EXPECTED shape of a Client-initiated cycle (Host is
					 * about to catch up to the solicitation already in
					 * flight) -- split out from the general rejection count
					 * so it isn't confused with a genuine problem (e.g.
					 * clientState stuck eClientArmed/eClientTransferring for
					 * some other reason). */
					if (pInst->clientState == eClientSelfArmed)
						{
							gDiagOnSelectRejectedSelfArmedCount++;
						}
				}
			return;
		}

	/* Claim eClientArmed before triggering pTransferStart/pReadyAssert
	 * below, not after: on the host-native null-loopback HW adapter (and,
	 * on real hardware, if the physical transfer completes and its ISRs
	 * run before this function returns), the whole transfer -- including
	 * onClockStart/onTransferComplete moving clientState on to
	 * eClientTransferring/eClientIdle -- can happen synchronously inside
	 * that pReadyAssert call. Assigning eClientArmed afterwards would
	 * stomp that already-correct later state back to a stale one. */
	pInst->clientState = eClientArmed;
	buildOutgoingFrame (pInst, pInst->txBuffer);
	teSpiTransportError startErr = pInst->prHw->pTransferStart (
	    pInst->prHw->pContext, pInst->txBuffer, pInst->rxBuffer, SPI_TRANSPORT_FRAME_TOTAL_SIZE);

	if (startErr != eSpiTransportErrorNone)
		{
			/* Slave DMA never actually armed. Abandon the peeked chunk
			 * (leave it uncommitted for a clean retry) and go back to
			 * Idle without ever asserting NRDY low -- Host's existing
			 * 100ms ack-wait timeout already handles "Client never acked
			 * this attempt" correctly, so this degrades to that same,
			 * already-working retry path instead of needing a new one. */
			pInst->txCommitPending = false;
			pInst->clientState     = eClientIdle;
			spiTransportChannelNotifyEvent (&pInst->channels, eSpiTransportEventErrorDmaFailure);
			return;
		}
	gDiagOnSelectArmedCount++;
	pInst->prHw->pReadyAssert (pInst->prHw->pContext, false);
}

static void
onClockStart (void *pCoreCtx)
{
	trSpiTransportInstance *pInst = (trSpiTransportInstance *)pCoreCtx;

	if (pInst->role != eSpiTransportRoleClient)
		{
			return;
		}

	pInst->prHw->pReadyAssert (pInst->prHw->pContext, true);
	pInst->clientState = eClientTransferring;
}

/* Client only. If idle and something is queued, arm the Client's own DMA
 * and drop NRDY while NSS is still HIGH, soliciting Host -- see
 * hostArmClientInitiatedIfIdle() for how Host disambiguates this from its
 * own request cycle (no new wire signal needed). Task-context only (called
 * from clientServiceTick()), so the critical section here is what keeps
 * onSelectEvent()'s ISR-context clientState!=eClientIdle check race-free
 * against this claim -- unlike Host's symmetric pair, only this one writer
 * needs the section, since onSelectEvent is the sole other writer and it
 * only ever runs with this masked out while the section is held. */
static void
clientArmSelfInitiateIfIdle (trSpiTransportInstance *pInst, uint32_t now)
{
	if (!pInst->clientSelfInitEnabled)
		{
			return; /* disabled (the default) -- see spiTransportSetClientSelfInitEnabled() */
		}

	bool haveSomethingQueued = pInst->pendingHello || pInst->pendingHelloAck
	                           || spiTransportChannelHasPending (&pInst->channels);
	/* Same backpressure reasoning as onSelectEvent's ring-full check: don't
	 * solicit Host for a reply this Client has no room to receive. */
	bool ringFull = (pInst->rxRingHead - pInst->rxRingTail) >= SPI_TRANSPORT_RX_RING_DEPTH;

	pInst->prOs->pCriticalEnter (pInst->prOs->pContext);
	bool shouldArm = (pInst->clientState == eClientIdle) && haveSomethingQueued && !ringFull;
	if (shouldArm)
		{
			pInst->clientState          = eClientSelfArmed;
			pInst->clientSelfArmStartMs = now;
		}
	pInst->prOs->pCriticalExit (pInst->prOs->pContext);

	if (!shouldArm)
		{
			if (ringFull && (pInst->clientState == eClientIdle) && haveSomethingQueued)
				{
					gDiagRxRingFullRejectedCount++;
				}
			return;
		}
	gDiagClientSelfArmAttemptCount++;
	buildOutgoingFrame (pInst, pInst->txBuffer);
	teSpiTransportError startErr = pInst->prHw->pTransferStart (
	    pInst->prHw->pContext, pInst->txBuffer, pInst->rxBuffer, SPI_TRANSPORT_FRAME_TOTAL_SIZE);

	if (startErr != eSpiTransportErrorNone)
		{
			/* Same degrade-to-existing-retry-path reasoning as
			 * onSelectEvent's own failure branch: abandon the peeked chunk,
			 * go back to Idle without ever asserting NRDY low, and let the
			 * next tick's opportunity retry. */
			pInst->txCommitPending = false;
			pInst->clientState     = eClientIdle;
			spiTransportChannelNotifyEvent (&pInst->channels, eSpiTransportEventErrorDmaFailure);
			return;
		}
	pInst->prHw->pReadyAssert (pInst->prHw->pContext,
	                           false); /* solicit: NRDY low, NSS still high */
}

/* Client only. Mirrors hostServiceTick(): services the self-init watchdog
 * (eClientSelfArmed timeout) and otherwise attempts a new self-initiation
 * when idle. Called from spiTransportTick() for the Client role. */
static void
clientServiceTick (trSpiTransportInstance *pInst, uint32_t now)
{
	if (pInst->clientState == eClientSelfArmed)
		{
			if ((now - pInst->clientSelfArmStartMs) >= SPI_TRANSPORT_CLIENT_SELF_ARM_TIMEOUT_MS)
				{
					/* Host never noticed/clocked this solicitation -- self-heal
					 * now rather than waiting out the far coarser 1.5s
					 * disconnect backstop (doDisconnect()'s own generic
					 * clientState!=eClientIdle handling there remains a
					 * second-layer defense if this ever doesn't fire, e.g. if
					 * the platform starves this tick badly enough). pAbort()
					 * BEFORE resetting clientState/NRDY -- same ordering rule
					 * as doDisconnect()'s Client branch, for the same reason:
					 * closes the window where a concurrent
					 * onSelectEvent/self-init claim could re-arm against
					 * hardware still mid-abort. */
					gDiagClientSelfArmTimeoutCount++;
					pInst->prHw->pAbort (pInst->prHw->pContext);
					pInst->clientState = eClientIdle;
					pInst->prHw->pReadyAssert (pInst->prHw->pContext, true);
					spiTransportChannelNotifyEvent (&pInst->channels,
					                                eSpiTransportEventErrorDmaTimeout);
				}
			return; /* still self-armed (not yet timed out): don't also retry */
		}

	if (pInst->clientState != eClientIdle)
		{
			return; /* eClientArmed/eClientTransferring: reacting to a real Host request */
		}

	clientArmSelfInitiateIfIdle (pInst, now);
}

static void
onReadyEvent (void *pCoreCtx, bool high)
{
	trSpiTransportInstance *pInst = (trSpiTransportInstance *)pCoreCtx;

	if (pInst->role != eSpiTransportRoleHost)
		{
			return;
		}

	/* Interrupt-driven NRDY watch: acts immediately on the edge rather than
     * waiting for the next spiTransportTick() poll -- the tick-driven path
     * (hostServiceTick) remains correct on its own and is what a platform
     * with no NRDY EXTI (or an adapter that leaves pOnReadyEvent unfired)
     * relies on exclusively; this is a pure latency optimization. */
	if (!high)
		{
			/* Falling edge: either Client's ack for Host's own pending
			 * request (hostState==eHostWaitingAck), or Client self-
			 * initiating while Host was idle (hostState==eHostIdle) --
			 * mutually exclusive by construction (see
			 * hostArmClientInitiatedIfIdle()'s doc comment), so branch on
			 * which rather than trying both (calling the wrong one is a
			 * harmless no-op either way, but would pollute that path's own
			 * [DIAG] rejection counter with an unrelated cause). */
			if (pInst->hostState == eHostWaitingAck)
				{
					hostArmTransferIfAcked (pInst);
				}
			else
				{
					hostArmClientInitiatedIfIdle (pInst);
				}
		}
	else
		{
			/* Rising edge: Client became available again. */
			hostIssueRequestIfReady (pInst, pInst->prOs->pTickGet (pInst->prOs->pContext));
		}
}

teSpiTransportError
spiTransportInit (const trSpiTransportConfig *prConfig, thSpiTransport *phTransport)
{
	if ((prConfig == NULL) || (prConfig->prOs == NULL) || (prConfig->prHw == NULL)
	    || (phTransport == NULL))
		{
			return eSpiTransportErrorInvalidParam;
		}

	crc16Init ();

	trSpiTransportInstance *pInst = NULL;
	for (uint8_t i = 0; i < SPI_TRANSPORT_INSTANCES_MAX; i++)
		{
			if (!gInstances[i].inUse)
				{
					pInst = &gInstances[i];
					break;
				}
		}
	if (pInst == NULL)
		{
			return eSpiTransportErrorBusy; /* pool exhausted */
		}

	memset (pInst, 0, sizeof (*pInst));

	pInst->inUse = true;
	pInst->role  = prConfig->role;
	pInst->prOs  = prConfig->prOs;
	pInst->prHw  = prConfig->prHw;

	spiTransportChannelTableInit (&pInst->channels);
	pInst->linkState     = eSpiTransportLinkDisconnected;
	pInst->lastTxChannel = SPI_TRANSPORT_CHANNELS_MAX - 1u; /* round-robin starts at 0 */
	pInst->epoch         = pInst->prOs->pTickGet (pInst->prOs->pContext);

	spiTransportHwSetCallbacks (pInst->prHw, onTransferComplete, onSelectEvent, onReadyEvent,
	                            onClockStart, pInst);

	*phTransport = pInst;
	return eSpiTransportErrorNone;
}

teSpiTransportError
spiTransportStart (thSpiTransport hTransport)
{
	trSpiTransportInstance *pInst = (trSpiTransportInstance *)hTransport;
	if ((pInst == NULL) || !pInst->inUse)
		{
			return eSpiTransportErrorInvalidParam;
		}

	uint32_t now = pInst->prOs->pTickGet (pInst->prOs->pContext);

	pInst->peerEpochKnown = false;
	resetForHandshake (pInst);
	pInst->txSeq          = 0;
	pInst->rxLastSeq      = 0;
	pInst->lastSendTickMs = now;
	pInst->lastRecvTickMs = now;
	pInst->hostState      = eHostIdle;
	pInst->clientState    = eClientIdle;

	pInst->rxRingHead = 0u;
	pInst->rxRingTail = 0u;

	/* Force the physical line back to idle-high regardless of whatever
	 * level it was left at (e.g. a prior spiTransportStop() call, or a
	 * board's power-on GPIO default) -- otherwise a Start() that follows a
	 * mid-transfer Stop() (or the very first Start() before any GPIO init
	 * guarantee) can begin operation with the peer seeing a stale low
	 * NSS/NRDY that this session's state machine never actually asserted. */
	if (pInst->role == eSpiTransportRoleHost)
		{
			pInst->prHw->pSelectAssert (pInst->prHw->pContext, true);
		}
	else
		{
			pInst->prHw->pReadyAssert (pInst->prHw->pContext, true);
		}

	return eSpiTransportErrorNone;
}

void
spiTransportStop (thSpiTransport hTransport)
{
	trSpiTransportInstance *pInst = (trSpiTransportInstance *)hTransport;
	if ((pInst == NULL) || !pInst->inUse)
		{
			return;
		}

	/* Same un-wedge requirement as doDisconnect()'s backstop -- Stop() can
	 * be called mid-transfer (see the fault-injection "reset during
	 * transfer" mode in docs/TestPlan.md), and simply resetting
	 * hostState/clientState below without releasing the physical line or
	 * aborting a genuinely-armed peripheral would leave NSS/NRDY latched
	 * and the SPI/DMA peripheral still busy for whatever runs next. Abort
	 * before flipping state, same ordering reason as doDisconnect(). */
	if (pInst->role == eSpiTransportRoleHost)
		{
			if (pInst->hostState == eHostTransferring)
				{
					pInst->prHw->pAbort (pInst->prHw->pContext);
				}
			pInst->prHw->pSelectAssert (pInst->prHw->pContext, true);
		}
	else
		{
			if (pInst->clientState != eClientIdle)
				{
					pInst->prHw->pAbort (pInst->prHw->pContext);
				}
			pInst->prHw->pReadyAssert (pInst->prHw->pContext, true);
		}

	pInst->linkState   = eSpiTransportLinkDisconnected;
	pInst->hostState   = eHostIdle;
	pInst->clientState = eClientIdle;
	spiTransportChannelResetAll (&pInst->channels);
}

teSpiTransportError
spiTransportRegisterChannel (thSpiTransport hTransport, uint8_t channel,
                             tpSpiTransportRxCallback pRxCallback,
                             tpSpiTransportEventCallback pEventCallback, void *pContext)
{
	trSpiTransportInstance *pInst = (trSpiTransportInstance *)hTransport;
	if ((pInst == NULL) || !pInst->inUse)
		{
			return eSpiTransportErrorInvalidParam;
		}
	return spiTransportChannelRegister (&pInst->channels, channel, pRxCallback, pEventCallback,
	                                    pContext);
}

teSpiTransportError
spiTransportDeregisterChannel (thSpiTransport hTransport, uint8_t channel)
{
	trSpiTransportInstance *pInst = (trSpiTransportInstance *)hTransport;
	if ((pInst == NULL) || !pInst->inUse)
		{
			return eSpiTransportErrorInvalidParam;
		}
	return spiTransportChannelDeregister (&pInst->channels, channel);
}

teSpiTransportError
spiTransportSend (thSpiTransport hTransport, uint8_t channel, const uint8_t *pBuffer,
                  uint16_t length, bool ackRequired)
{
	trSpiTransportInstance *pInst = (trSpiTransportInstance *)hTransport;
	if ((pInst == NULL) || !pInst->inUse)
		{
			return eSpiTransportErrorInvalidParam;
		}
	return spiTransportChannelQueueTx (&pInst->channels, channel, pBuffer, length, ackRequired);
}

teSpiTransportLinkState
spiTransportGetLinkState (thSpiTransport hTransport)
{
	trSpiTransportInstance *pInst = (trSpiTransportInstance *)hTransport;
	if ((pInst == NULL) || !pInst->inUse)
		{
			return eSpiTransportLinkDisconnected;
		}
	return pInst->linkState;
}

void
spiTransportSetClientSelfInitEnabled (thSpiTransport hTransport, bool enabled)
{
	trSpiTransportInstance *pInst = (trSpiTransportInstance *)hTransport;
	if ((pInst == NULL) || !pInst->inUse || (pInst->role != eSpiTransportRoleClient))
		{
			return;
		}
	pInst->clientSelfInitEnabled = enabled;
}

bool
spiTransportIsClientSelfInitEnabled (thSpiTransport hTransport)
{
	trSpiTransportInstance *pInst = (trSpiTransportInstance *)hTransport;
	if ((pInst == NULL) || !pInst->inUse)
		{
			return false;
		}
	return pInst->clientSelfInitEnabled;
}

void
spiTransportTick (thSpiTransport hTransport)
{
	trSpiTransportInstance *pInst = (trSpiTransportInstance *)hTransport;
	if ((pInst == NULL) || !pInst->inUse)
		{
			return;
		}

	/* Consumer side of the ring: drain everything queued, not just one
	 * frame, so a burst of back-to-back completions (see onTransferComplete)
	 * doesn't leave later ones waiting for a whole extra tick. Reads
	 * rxRingHead (written only by the producer/ISR) fresh each iteration --
	 * safe without a lock for the same single-writer-per-counter reason as
	 * the producer side, and picks up anything that arrived mid-drain for
	 * free. Strict FIFO: tail only ever advances by exactly one slot at a
	 * time, in arrival order. */
	while (pInst->rxRingTail != pInst->rxRingHead)
		{
			uint32_t tail = pInst->rxRingTail;
			processIncomingFrame (pInst, pInst->rxRingBuffer[tail % SPI_TRANSPORT_RX_RING_DEPTH]);
			pInst->rxRingTail = tail + 1u;
		}

	uint32_t now = pInst->prOs->pTickGet (pInst->prOs->pContext);

	if ((pInst->linkState != eSpiTransportLinkDisconnected)
	    && ((now - pInst->lastRecvTickMs) >= SPI_TRANSPORT_DISCONNECT_MS))
		{
			doDisconnect (pInst, now);
		}

	if (pInst->role == eSpiTransportRoleHost)
		{
			hostServiceTick (pInst, now);
		}
	else
		{
			clientServiceTick (pInst, now);
		}
}

uint32_t
spiTransportDebugRxOverwriteCount (thSpiTransport hTransport)
{
	trSpiTransportInstance *pInst = (trSpiTransportInstance *)hTransport;
	if ((pInst == NULL) || !pInst->inUse)
		{
			return 0u;
		}
	return pInst->rxOverwriteCount;
}

void
spiTransportDebugTxSeqCounts (thSpiTransport hTransport, uint32_t *pBuiltCount,
                              uint32_t *pAdvanceCount, uint16_t *pCurrentTxSeq)
{
	trSpiTransportInstance *pInst = (trSpiTransportInstance *)hTransport;
	if ((pInst == NULL) || !pInst->inUse)
		{
			if (pBuiltCount != NULL)
				{
					*pBuiltCount = 0u;
				}
			if (pAdvanceCount != NULL)
				{
					*pAdvanceCount = 0u;
				}
			if (pCurrentTxSeq != NULL)
				{
					*pCurrentTxSeq = 0u;
				}
			return;
		}
	if (pBuiltCount != NULL)
		{
			*pBuiltCount = gDiagTxFrameBuiltCount;
		}
	if (pAdvanceCount != NULL)
		{
			*pAdvanceCount = gDiagTxSeqAdvanceCount;
		}
	if (pCurrentTxSeq != NULL)
		{
			*pCurrentTxSeq = pInst->txSeq;
		}
}

uint32_t
spiTransportDebugRxRingFullRejectedCount (thSpiTransport hTransport)
{
	trSpiTransportInstance *pInst = (trSpiTransportInstance *)hTransport;
	if ((pInst == NULL) || !pInst->inUse)
		{
			return 0u;
		}
	return gDiagRxRingFullRejectedCount;
}

void
spiTransportDebugClientArmCounts (thSpiTransport hTransport, uint32_t *pRejectedCount,
                                  uint32_t *pArmedCount)
{
	(void)hTransport;
	if (pRejectedCount != NULL)
		{
			*pRejectedCount = gDiagOnSelectRejectedCount;
		}
	if (pArmedCount != NULL)
		{
			*pArmedCount = gDiagOnSelectArmedCount;
		}
}

void
spiTransportDebugHostArmCounts (thSpiTransport hTransport, uint32_t *pRejectedCount,
                                uint32_t *pArmedCount)
{
	(void)hTransport;
	if (pRejectedCount != NULL)
		{
			*pRejectedCount = gDiagHostArmRejectedCount;
		}
	if (pArmedCount != NULL)
		{
			*pArmedCount = gDiagHostArmArmedCount;
		}
}

void
spiTransportDebugClientSelfInitCounts (thSpiTransport hTransport, uint32_t *pAttemptCount,
                                       uint32_t *pTimeoutCount, uint32_t *pHostArmedCount,
                                       uint32_t *pRejectedSelfArmedCount)
{
	(void)hTransport;
	if (pAttemptCount != NULL)
		{
			*pAttemptCount = gDiagClientSelfArmAttemptCount;
		}
	if (pTimeoutCount != NULL)
		{
			*pTimeoutCount = gDiagClientSelfArmTimeoutCount;
		}
	if (pHostArmedCount != NULL)
		{
			*pHostArmedCount = gDiagHostClientInitArmedCount;
		}
	if (pRejectedSelfArmedCount != NULL)
		{
			*pRejectedSelfArmedCount = gDiagOnSelectRejectedSelfArmedCount;
		}
}

void
spiTransportDebugLastGap (thSpiTransport hTransport, uint16_t *pExpected, uint16_t *pActual,
                          uint32_t *pDuplicateCount, uint32_t *pLossCount, uint32_t *pOtherCount)
{
	trSpiTransportInstance *pInst = (trSpiTransportInstance *)hTransport;
	if ((pInst == NULL) || !pInst->inUse)
		{
			return;
		}
	if (pExpected != NULL)
		{
			*pExpected = pInst->lastGapExpected;
		}
	if (pActual != NULL)
		{
			*pActual = pInst->lastGapActual;
		}
	if (pDuplicateCount != NULL)
		{
			*pDuplicateCount = pInst->gapDuplicateCount;
		}
	if (pLossCount != NULL)
		{
			*pLossCount = pInst->gapLossCount;
		}
	if (pOtherCount != NULL)
		{
			*pOtherCount = pInst->gapOtherCount;
		}
}

void
spiTransportDebugEpoch (thSpiTransport hTransport, uint32_t *pOwnEpoch, uint32_t *pPeerEpoch,
                        bool *pPeerEpochKnown)
{
	trSpiTransportInstance *pInst = (trSpiTransportInstance *)hTransport;
	if ((pInst == NULL) || !pInst->inUse)
		{
			return;
		}
	if (pOwnEpoch != NULL)
		{
			*pOwnEpoch = pInst->epoch;
		}
	if (pPeerEpoch != NULL)
		{
			*pPeerEpoch = pInst->peerEpoch;
		}
	if (pPeerEpochKnown != NULL)
		{
			*pPeerEpochKnown = pInst->peerEpochKnown;
		}
}
