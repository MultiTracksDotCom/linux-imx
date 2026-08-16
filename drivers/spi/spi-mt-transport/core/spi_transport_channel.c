//******************************************************************************
// @file      : spi_transport_channel.c
// @brief     : Channel registration table, RX reassembly, per-channel
//              single-slot TX queue. See spi_transport_channel.h.
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

#include "spi_transport/spi_transport_channel.h"

#include <string.h>

void
spiTransportChannelTableInit (trSpiTransportChannelTable *prTable)
{
	memset (prTable, 0, sizeof (*prTable));
}

teSpiTransportError
spiTransportChannelRegister (trSpiTransportChannelTable *prTable, uint8_t channel,
                             tpSpiTransportRxCallback pRxCallback,
                             tpSpiTransportEventCallback pEventCallback, void *pContext)
{
	if ((channel == SPI_TRANSPORT_CHANNEL_CONTROL) || (channel >= SPI_TRANSPORT_CHANNELS_MAX))
		{
			return eSpiTransportErrorInvalidChannel;
		}
	if (prTable->slots[channel].registered)
		{
			return eSpiTransportErrorAlreadyRegistered;
		}

	trSpiTransportChannelSlot *pSlot = &prTable->slots[channel];
	memset (pSlot, 0, sizeof (*pSlot));
	pSlot->registered     = true;
	pSlot->pRxCallback    = pRxCallback;
	pSlot->pEventCallback = pEventCallback;
	pSlot->pContext       = pContext;

	return eSpiTransportErrorNone;
}

teSpiTransportError
spiTransportChannelDeregister (trSpiTransportChannelTable *prTable, uint8_t channel)
{
	if ((channel == SPI_TRANSPORT_CHANNEL_CONTROL) || (channel >= SPI_TRANSPORT_CHANNELS_MAX))
		{
			return eSpiTransportErrorInvalidChannel;
		}
	if (!prTable->slots[channel].registered)
		{
			return eSpiTransportErrorNotRegistered;
		}

	memset (&prTable->slots[channel], 0, sizeof (prTable->slots[channel]));
	return eSpiTransportErrorNone;
}

teSpiTransportError
spiTransportChannelQueueTx (trSpiTransportChannelTable *prTable, uint8_t channel,
                            const uint8_t *pBuffer, uint16_t length, bool ackRequired)
{
	/* Channel 0 is transport-internal (HELLO/HELLO_ACK/FILLER) -- matches
	 * spiTransportChannelRegister()'s own guard. Without this, application
	 * data queued on channel 0 would collide with control-frame traffic and
	 * corrupt the handshake/state machine (spiTransportChannelNextTx() has
	 * no way to distinguish the two once queued). */
	if ((channel == SPI_TRANSPORT_CHANNEL_CONTROL) || (channel >= SPI_TRANSPORT_CHANNELS_MAX))
		{
			return eSpiTransportErrorInvalidChannel;
		}
	if (length > SPI_TRANSPORT_CHANNEL_MESSAGE_MAX)
		{
			return eSpiTransportErrorInvalidParam;
		}
	if ((pBuffer == NULL) && (length > 0u))
		{
			return eSpiTransportErrorInvalidParam;
		}

	trSpiTransportChannelSlot *pSlot = &prTable->slots[channel];
	if (pSlot->txPending)
		{
			return eSpiTransportErrorBusy;
		}

	pSlot->txPending     = true;
	pSlot->pTxBuffer     = pBuffer;
	pSlot->txLength      = length;
	pSlot->txOffset      = 0;
	pSlot->txAckRequired = ackRequired;

	return eSpiTransportErrorNone;
}

bool
spiTransportChannelNextTx (trSpiTransportChannelTable *prTable, uint8_t startAfterChannel,
                           uint8_t *pOutChannel, uint8_t *pOutPayload, uint16_t *pOutLength,
                           uint8_t *pOutFlags)
{
	uint8_t start = (uint8_t)((startAfterChannel + 1u) % SPI_TRANSPORT_CHANNELS_MAX);

	for (uint8_t i = 0; i < SPI_TRANSPORT_CHANNELS_MAX; i++)
		{
			uint8_t channel                  = (uint8_t)((start + i) % SPI_TRANSPORT_CHANNELS_MAX);
			trSpiTransportChannelSlot *pSlot = &prTable->slots[channel];

			if (!pSlot->txPending)
				{
					continue;
				}

			uint16_t remaining = (uint16_t)(pSlot->txLength - pSlot->txOffset);
			uint16_t chunk     = (remaining < SPI_TRANSPORT_FRAME_PAYLOAD_SIZE)
			                         ? remaining
			                         : SPI_TRANSPORT_FRAME_PAYLOAD_SIZE;

			uint8_t flags = 0;
			if (pSlot->txOffset == 0u)
				{
					flags = (uint8_t)(flags | SPI_TRANSPORT_FLAG_START);
				}
			bool isLastChunk = (uint16_t)(pSlot->txOffset + chunk) >= pSlot->txLength;
			if (isLastChunk)
				{
					flags = (uint8_t)(flags | SPI_TRANSPORT_FLAG_END);
				}
			if (pSlot->txAckRequired)
				{
					flags = (uint8_t)(flags | SPI_TRANSPORT_FLAG_ACK_REQUIRED);
				}

			if (chunk > 0u)
				{
					memcpy (pOutPayload, &pSlot->pTxBuffer[pSlot->txOffset], chunk);
				}

			*pOutChannel = channel;
			*pOutLength  = chunk;
			*pOutFlags   = flags;
			return true;
		}

	return false;
}

void
spiTransportChannelCommitTx (trSpiTransportChannelTable *prTable, uint8_t channel,
                             uint16_t chunkLen, bool wasLastChunk)
{
	if (channel >= SPI_TRANSPORT_CHANNELS_MAX)
		{
			return;
		}

	trSpiTransportChannelSlot *pSlot = &prTable->slots[channel];
	pSlot->txOffset                  = (uint16_t)(pSlot->txOffset + chunkLen);

	if (wasLastChunk)
		{
			pSlot->txPending = false;
			pSlot->pTxBuffer = NULL;
		}
}

bool
spiTransportChannelHasPending (const trSpiTransportChannelTable *prTable)
{
	for (uint8_t channel = 0; channel < SPI_TRANSPORT_CHANNELS_MAX; channel++)
		{
			if (prTable->slots[channel].txPending)
				{
					return true;
				}
		}

	return false;
}

void
spiTransportChannelDispatchRx (trSpiTransportChannelTable *prTable, uint8_t channel,
                               const uint8_t *pPayload, uint16_t length, uint8_t flags)
{
	if (channel >= SPI_TRANSPORT_CHANNELS_MAX)
		{
			return;
		}

	trSpiTransportChannelSlot *pSlot = &prTable->slots[channel];
	if (!pSlot->registered)
		{
			return; /* unregistered channel: silently dropped (counted by the caller, if desired) */
		}

	if (flags & SPI_TRANSPORT_FLAG_START)
		{
			pSlot->rxInProgress = true;
			pSlot->rxOffset     = 0;
		}

	if (!pSlot->rxInProgress)
		{
			return; /* END/middle chunk arrived with no START seen yet (e.g. post-reconnect) */
		}

	if ((uint32_t)pSlot->rxOffset + length > SPI_TRANSPORT_CHANNEL_MESSAGE_MAX)
		{
			pSlot->rxInProgress = false; /* oversized message: abandon and drop */
			return;
		}

	if (length > 0u)
		{
			memcpy (&pSlot->rxBuffer[pSlot->rxOffset], pPayload, length);
			pSlot->rxOffset = (uint16_t)(pSlot->rxOffset + length);
		}

	if (flags & SPI_TRANSPORT_FLAG_END)
		{
			pSlot->rxInProgress = false;
			if (pSlot->pRxCallback != NULL)
				{
					pSlot->pRxCallback (pSlot->pContext, channel, pSlot->rxBuffer, pSlot->rxOffset,
					                    flags);
				}
		}
}

void
spiTransportChannelResetAll (trSpiTransportChannelTable *prTable)
{
	for (uint8_t channel = 0; channel < SPI_TRANSPORT_CHANNELS_MAX; channel++)
		{
			trSpiTransportChannelSlot *pSlot = &prTable->slots[channel];
			pSlot->txPending                 = false;
			pSlot->pTxBuffer                 = NULL;
			pSlot->txOffset                  = 0;
			pSlot->rxInProgress              = false;
			pSlot->rxOffset                  = 0;
		}
}

void
spiTransportChannelNotifyEvent (trSpiTransportChannelTable *prTable, teSpiTransportEvent eEvent)
{
	for (uint8_t channel = 0; channel < SPI_TRANSPORT_CHANNELS_MAX; channel++)
		{
			trSpiTransportChannelSlot *pSlot = &prTable->slots[channel];
			if (pSlot->registered && (pSlot->pEventCallback != NULL))
				{
					pSlot->pEventCallback (pSlot->pContext, eEvent);
				}
		}
}
