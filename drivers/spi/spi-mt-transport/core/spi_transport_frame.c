//******************************************************************************
// @file      : spi_transport_frame.c
// @brief     : Wire-frame encode/decode. Explicit little-endian codec, not a
//              packed struct overlay -- see spi_transport_frame.h.
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

#include "spi_transport/spi_transport_frame.h"

#include <string.h>

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

teSpiTransportError
spiTransportFrameEncode (const trSpiTransportFrame *prFrame, const uint8_t *pPayload,
                         uint8_t *pOutBuffer)
{
	if ((prFrame == NULL) || (pOutBuffer == NULL))
		{
			return eSpiTransportErrorInvalidParam;
		}
	if (prFrame->payloadLength > SPI_TRANSPORT_FRAME_PAYLOAD_SIZE)
		{
			return eSpiTransportErrorInvalidParam;
		}
	if ((pPayload == NULL) && (prFrame->payloadLength > 0u))
		{
			return eSpiTransportErrorInvalidParam;
		}

	putU16 (&pOutBuffer[0], SPI_TRANSPORT_FRAME_MAGIC);
	pOutBuffer[2] = prFrame->version;
	pOutBuffer[3] = prFrame->channel;
	putU16 (&pOutBuffer[4], prFrame->seq);
	putU16 (&pOutBuffer[6], prFrame->ack);
	pOutBuffer[8] = prFrame->flags;
	pOutBuffer[9] = 0; /* reserved */
	putU16 (&pOutBuffer[10], prFrame->payloadLength);
	putU16 (&pOutBuffer[12], crc16 (pOutBuffer, SPI_TRANSPORT_FRAME_HEADER_SIZE - 2u));

	uint8_t *pPayloadRegion = &pOutBuffer[SPI_TRANSPORT_FRAME_HEADER_SIZE];
	if (prFrame->payloadLength > 0u)
		{
			memcpy (pPayloadRegion, pPayload, prFrame->payloadLength);
		}
	if (prFrame->payloadLength < SPI_TRANSPORT_FRAME_PAYLOAD_SIZE)
		{
			memset (&pPayloadRegion[prFrame->payloadLength], 0,
			        (size_t)(SPI_TRANSPORT_FRAME_PAYLOAD_SIZE - prFrame->payloadLength));
		}

	uint16_t payloadCrc = crc16 (pPayloadRegion, prFrame->payloadLength);
	putU16 (&pOutBuffer[SPI_TRANSPORT_FRAME_HEADER_SIZE + SPI_TRANSPORT_FRAME_PAYLOAD_SIZE],
	        payloadCrc);

	return eSpiTransportErrorNone;
}

bool
spiTransportFrameHeaderCrcOk (const uint8_t *pInBuffer)
{
	if (pInBuffer == NULL)
		{
			return false;
		}
	uint16_t expected = getU16 (&pInBuffer[SPI_TRANSPORT_FRAME_HEADER_SIZE - 2u]);
	uint16_t actual   = crc16 (pInBuffer, SPI_TRANSPORT_FRAME_HEADER_SIZE - 2u);
	return expected == actual;
}

bool
spiTransportFramePayloadCrcOk (const uint8_t *pInBuffer)
{
	if (pInBuffer == NULL)
		{
			return false;
		}
	uint16_t payloadLength = getU16 (&pInBuffer[10]);
	if (payloadLength > SPI_TRANSPORT_FRAME_PAYLOAD_SIZE)
		{
			return false;
		}

	const uint8_t *pPayloadRegion = &pInBuffer[SPI_TRANSPORT_FRAME_HEADER_SIZE];
	uint16_t expected
	    = getU16 (&pInBuffer[SPI_TRANSPORT_FRAME_HEADER_SIZE + SPI_TRANSPORT_FRAME_PAYLOAD_SIZE]);
	uint16_t actual = crc16 (pPayloadRegion, payloadLength);
	return expected == actual;
}

teSpiTransportError
spiTransportFrameDecode (const uint8_t *pInBuffer, trSpiTransportFrame *prFrame)
{
	if ((pInBuffer == NULL) || (prFrame == NULL))
		{
			return eSpiTransportErrorInvalidParam;
		}

	uint16_t magic = getU16 (&pInBuffer[0]);
	if (magic != SPI_TRANSPORT_FRAME_MAGIC)
		{
			return eSpiTransportErrorInvalidParam;
		}

	if (!spiTransportFrameHeaderCrcOk (pInBuffer))
		{
			return eSpiTransportErrorHardwareFailure;
		}

	uint16_t payloadLength = getU16 (&pInBuffer[10]);
	if (payloadLength > SPI_TRANSPORT_FRAME_PAYLOAD_SIZE)
		{
			return eSpiTransportErrorInvalidParam;
		}

	if (!spiTransportFramePayloadCrcOk (pInBuffer))
		{
			return eSpiTransportErrorHardwareFailure;
		}

	prFrame->magic         = magic;
	prFrame->version       = pInBuffer[2];
	prFrame->channel       = pInBuffer[3];
	prFrame->seq           = getU16 (&pInBuffer[4]);
	prFrame->ack           = getU16 (&pInBuffer[6]);
	prFrame->flags         = pInBuffer[8];
	prFrame->payloadLength = payloadLength;
	prFrame->pPayload      = &pInBuffer[SPI_TRANSPORT_FRAME_HEADER_SIZE];

	return eSpiTransportErrorNone;
}
