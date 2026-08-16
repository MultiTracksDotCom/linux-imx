//******************************************************************************
// @file      : spi_transport_channel.h
// @brief     : Internal channel registration table, RX reassembly, and
//              per-channel single-slot TX queue. Used by spi_transport.c
//              only -- not part of the public API (see spi_transport.h).
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

#ifndef SPI_TRANSPORT_CHANNEL_H
#define SPI_TRANSPORT_CHANNEL_H

#include <stdbool.h>
#include <stdint.h>

#include "spi_transport/spi_transport.h"
#include "spi_transport/spi_transport_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct _trSpiTransportChannelTable trSpiTransportChannelTable;

void spiTransportChannelTableInit (trSpiTransportChannelTable *prTable);

teSpiTransportError spiTransportChannelRegister (trSpiTransportChannelTable *prTable,
                                                 uint8_t channel,
                                                 tpSpiTransportRxCallback pRxCallback,
                                                 tpSpiTransportEventCallback pEventCallback,
                                                 void *pContext);
teSpiTransportError spiTransportChannelDeregister (trSpiTransportChannelTable *prTable,
                                                   uint8_t channel);

/// @brief Queue `length` bytes for `channel`. Buffer is borrowed (not
///        copied) -- caller must keep it valid until fully sent; returns
///        eSpiTransportErrorInvalidChannel for channel 0 (transport-internal,
///        see SPI_TRANSPORT_CHANNEL_CONTROL) or channel >=
///        SPI_TRANSPORT_CHANNELS_MAX, eSpiTransportErrorBusy if a previous
///        message on this channel hasn't finished, eSpiTransportErrorInvalidParam
///        if length exceeds SPI_TRANSPORT_CHANNEL_MESSAGE_MAX or pBuffer is
///        NULL with a nonzero length.
teSpiTransportError spiTransportChannelQueueTx (trSpiTransportChannelTable *prTable,
                                                uint8_t channel, const uint8_t *pBuffer,
                                                uint16_t length, bool ackRequired);

/// @brief Round-robin pick of the next channel with TX data queued, starting
///        the search just after `startAfterChannel` (caller passes the last
///        channel serviced, or SPI_TRANSPORT_CHANNELS_MAX to start at 0).
///        On a hit, copies up to SPI_TRANSPORT_FRAME_PAYLOAD_SIZE bytes into
///        pOutPayload and sets *pOutChannel/*pOutFlags/*pOutLength. Returns
///        false if no channel has anything queued.
///
///        Deliberately does NOT advance the channel's send offset or clear
///        txPending -- this only PEEKS the next chunk to build into a
///        frame. The caller must call spiTransportChannelCommitTx() once
///        the physical transfer carrying that chunk is confirmed to have
///        actually completed, not before. Committing at peek time (the
///        original design) meant any transfer that failed to arm or never
///        completed silently and permanently lost that chunk -- confirmed
///        live on hardware as a real, if hard to isolate, contributor to
///        the peer's sequence-gap counter (txSeq was already consumed for
///        a frame that never actually reached the peer).
bool spiTransportChannelNextTx (trSpiTransportChannelTable *prTable, uint8_t startAfterChannel,
                                uint8_t *pOutChannel, uint8_t *pOutPayload, uint16_t *pOutLength,
                                uint8_t *pOutFlags);

/// @brief Confirm the chunk most recently returned by spiTransportChannelNextTx
///        for `channel` actually went out -- advances that channel's send
///        offset by `chunkLen` and, if `wasLastChunk`, clears txPending
///        (frees the single in-flight slot for a new spiTransportChannelQueueTx
///        call). Never call this for a transfer that failed to arm or never
///        completed -- simply not calling it is the correct "abandon this
///        attempt, retry the same unconsumed chunk next time" behavior.
void spiTransportChannelCommitTx (trSpiTransportChannelTable *prTable, uint8_t channel,
                                  uint16_t chunkLen, bool wasLastChunk);

/// @brief True if any channel has a TX message queued (used by the Host's
///        heartbeat check to decide whether it needs to issue a request
///        before the (effectively every-tick) backstop timer fires).
bool spiTransportChannelHasPending (const trSpiTransportChannelTable *prTable);

/// @brief Reassemble an inbound chunk (per the START/END flags) and, once a
///        full message is complete, invoke the channel's RX callback in the
///        caller's context (task context -- never call from an ISR).
///        Silently drops+counts data for an unregistered channel.
void spiTransportChannelDispatchRx (trSpiTransportChannelTable *prTable, uint8_t channel,
                                    const uint8_t *pPayload, uint16_t length, uint8_t flags);

/// @brief Discard any in-progress RX reassembly and TX-in-flight state on
///        every channel -- called on every reconnect (epoch change), per
///        docs/ProtocolSpec.md's reconnect-baseline rule.
void spiTransportChannelResetAll (trSpiTransportChannelTable *prTable);

/// @brief Notify every registered channel's event callback (may be NULL) of
///        a link-wide event. Not channel-specific -- every subscriber sees
///        the same sequence.
void spiTransportChannelNotifyEvent (trSpiTransportChannelTable *prTable,
                                     teSpiTransportEvent eEvent);

typedef struct _trSpiTransportChannelSlot
{
	bool registered;
	tpSpiTransportRxCallback pRxCallback;
	tpSpiTransportEventCallback pEventCallback;
	void *pContext;

	/* TX (single in-flight message, borrowed buffer). */
	bool txPending;
	const uint8_t *pTxBuffer;
	uint16_t txLength;
	uint16_t txOffset;
	bool txAckRequired;

	/* RX reassembly (static buffer, no malloc). */
	bool rxInProgress;
	uint8_t rxBuffer[SPI_TRANSPORT_CHANNEL_MESSAGE_MAX];
	uint16_t rxOffset;
} trSpiTransportChannelSlot;

struct _trSpiTransportChannelTable
{
	trSpiTransportChannelSlot slots[SPI_TRANSPORT_CHANNELS_MAX];
};

#ifdef __cplusplus
}
#endif

#endif /* SPI_TRANSPORT_CHANNEL_H */
