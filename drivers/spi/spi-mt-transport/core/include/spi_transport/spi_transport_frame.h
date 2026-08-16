//******************************************************************************
// @file      : spi_transport_frame.h
// @brief     : Wire-frame encode/decode and CRC-16/CCITT-FALSE declarations.
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

#ifndef SPI_TRANSPORT_FRAME_H
#define SPI_TRANSPORT_FRAME_H

#include <stdbool.h>
#include <stdint.h>

#include "spi_transport/spi_transport_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

/// @brief Decoded view of a frame -- never overlaid directly on the DMA
///        buffer (see docs/ProtocolSpec.md "Frame layout" for why: explicit
///        codec functions avoid padding/aliasing hazards across compilers).
typedef struct _trSpiTransportFrame
{
	uint16_t magic;
	uint8_t version;
	uint8_t channel;
	uint16_t seq;
	uint16_t ack;
	uint8_t flags;
	uint16_t payloadLength;
	const uint8_t *pPayload; /* points into the caller-owned decode buffer, valid only
                                 until the next spiTransportFrameDecode() call on that buffer */
} trSpiTransportFrame;

/// @brief CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflect).
///        crc16Init() must be called once before any other CRC/frame call.
void crc16Init (void);
uint16_t crc16 (const uint8_t *pBuffer, uint16_t length);

/// @brief Encode prFrame plus payload bytes [0, payloadLength) into pOutBuffer
///        (must be exactly SPI_TRANSPORT_FRAME_TOTAL_SIZE bytes). Fills in
///        headerCrc/payloadCrc; prFrame->payloadLength bytes are copied from
///        pPayload, the rest of the payload region is zero-filled.
teSpiTransportError spiTransportFrameEncode (const trSpiTransportFrame *prFrame,
                                             const uint8_t *pPayload, uint8_t *pOutBuffer);

/// @brief Decode pInBuffer (exactly SPI_TRANSPORT_FRAME_TOTAL_SIZE bytes) into
///        *prFrame. prFrame->pPayload is set to point inside pInBuffer.
///        Returns eSpiTransportErrorInvalidParam on a magic mismatch,
///        eSpiTransportErrorHardwareFailure on a header or payload CRC
///        mismatch (caller distinguishes via the two CRC-check functions
///        below if it needs to tell header-CRC apart from payload-CRC
///        failures for event reporting).
teSpiTransportError spiTransportFrameDecode (const uint8_t *pInBuffer,
                                             trSpiTransportFrame *prFrame);

/// @brief Standalone checks, used by spiTransportFrameDecode() internally and
///        exposed so callers (and unit tests) can distinguish which CRC
///        failed without re-decoding.
bool spiTransportFrameHeaderCrcOk (const uint8_t *pInBuffer);
bool spiTransportFramePayloadCrcOk (const uint8_t *pInBuffer);

#ifdef __cplusplus
}
#endif

#endif /* SPI_TRANSPORT_FRAME_H */
