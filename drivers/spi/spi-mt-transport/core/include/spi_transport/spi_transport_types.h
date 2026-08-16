//******************************************************************************
// @file      : spi_transport_types.h
// @brief     : Wire-format constants, flags, error codes and role/state enums
//              for the SPI transport.
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

#ifndef SPI_TRANSPORT_TYPES_H
#define SPI_TRANSPORT_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* Frame layout (see docs/ProtocolSpec.md "Frame layout"):
 *   offset  size  field
 *   0       2     magic        (0x5AA5, little-endian)
 *   2       1     version
 *   3       1     channel
 *   4       2     seq
 *   6       2     ack
 *   8       1     flags
 *   9       1     reserved
 *   10      2     payloadLength
 *   12      2     headerCrc     (CRC-16/CCITT-FALSE over bytes 0-11)
 *   14      112   payload
 *   126     2     payloadCrc    (CRC-16/CCITT-FALSE over first payloadLength payload bytes)
 */
#define SPI_TRANSPORT_FRAME_MAGIC   (0x5AA5u)
#define SPI_TRANSPORT_FRAME_VERSION (1u)

#define SPI_TRANSPORT_FRAME_HEADER_SIZE  (14u)
#define SPI_TRANSPORT_FRAME_PAYLOAD_SIZE (112u)
#define SPI_TRANSPORT_FRAME_CRC_SIZE     (2u)
#define SPI_TRANSPORT_FRAME_TOTAL_SIZE                                                             \
	(SPI_TRANSPORT_FRAME_HEADER_SIZE + SPI_TRANSPORT_FRAME_PAYLOAD_SIZE                            \
	 + SPI_TRANSPORT_FRAME_CRC_SIZE)

#define SPI_TRANSPORT_CHANNEL_CONTROL (0u)
#define SPI_TRANSPORT_CHANNELS_MAX    (8u)

/* Max reassembled message size per channel (static buffer, no malloc). A
 * message larger than this cannot be sent/received -- spiTransportSend()
 * returns eSpiTransportErrorInvalidParam and an oversized inbound
 * (START-without-END-by-this-size) is dropped and counted, not delivered. */
#define SPI_TRANSPORT_CHANNEL_MESSAGE_MAX (512u)

/* Flags bitfield (offset 8). Bits 6-7 reserved, must be 0 on send. */
#define SPI_TRANSPORT_FLAG_START        (0x01u)
#define SPI_TRANSPORT_FLAG_END          (0x02u)
#define SPI_TRANSPORT_FLAG_ACK_REQUIRED (0x04u)
#define SPI_TRANSPORT_FLAG_ERROR        (0x08u)
#define SPI_TRANSPORT_FLAG_RESET        (0x10u)
#define SPI_TRANSPORT_FLAG_FILLER       (0x20u)

/* Channel-0 control message types. */
#define SPI_TRANSPORT_CTRL_HELLO     (1u)
#define SPI_TRANSPORT_CTRL_HELLO_ACK (2u)

/* Timing model (docs/ProtocolSpec.md "Connected/disconnected timing model").
 * Host re-issues a request itself, without waiting on this backstop,
 * whenever it has anything queued -- this heartbeat only governs how long
 * Host will go with NOTHING queued before issuing a request anyway (a
 * FILLER if still nothing to send by then), so real traffic is never
 * paced by this value. Set to effectively "every tick" (1ms) rather than
 * 500ms: Host must not sit idle for any noticeable stretch, even with
 * zero application traffic -- a fast, continuous heartbeat is cheap (a
 * FILLER frame) and keeps the link's actual round-trip latency close to
 * the physical transfer time instead of up to half a second.
 * SPI_TRANSPORT_DISCONNECT_MS is intentionally NOT scaled down to match --
 * it's a coarse "peer is genuinely gone" bound, not a heartbeat multiple,
 * and 1.5s remains the right tolerance for that regardless of how often
 * Host pings within it. */
#define SPI_TRANSPORT_HEARTBEAT_MS  (1u)
#define SPI_TRANSPORT_DISCONNECT_MS (1500u)

typedef enum
{
	eSpiTransportErrorNone = 0,
	eSpiTransportErrorInvalidParam,
	eSpiTransportErrorInvalidChannel,
	eSpiTransportErrorAlreadyRegistered,
	eSpiTransportErrorNotRegistered,
	eSpiTransportErrorBusy,
	eSpiTransportErrorNotConnected,
	eSpiTransportErrorHardwareFailure,
	eSpiTransportErrorTimeout,
} teSpiTransportError;

typedef enum
{
	eSpiTransportRoleHost   = 0,
	eSpiTransportRoleClient = 1,
} teSpiTransportRole;

typedef enum
{
	eSpiTransportLinkDisconnected = 0,
	eSpiTransportLinkHandshaking,
	eSpiTransportLinkConnected,
} teSpiTransportLinkState;

/* Link-wide events, delivered to every registered channel's event callback
 * (see spi_transport.h) -- connect/disconnect state changes and transport
 * errors are visible to every subscriber, not just RX data. */
typedef enum
{
	eSpiTransportEventConnected = 0,
	eSpiTransportEventDisconnected,
	eSpiTransportEventErrorHeaderCrc,
	eSpiTransportEventErrorPayloadCrc,
	eSpiTransportEventErrorSequenceGap,
	eSpiTransportEventErrorDmaFailure,
	eSpiTransportEventErrorDmaTimeout,
} teSpiTransportEvent;

#ifdef __cplusplus
}
#endif

#endif /* SPI_TRANSPORT_TYPES_H */
