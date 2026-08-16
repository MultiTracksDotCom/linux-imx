//******************************************************************************
// @file      : spi_transport_crc16.c
// @brief     : CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflect),
//              table-driven. New algorithm for this transport -- deliberately
//              not the shared CRC32 used elsewhere in this repo, see
//              docs/ProtocolSpec.md "CRC choice & duplication tradeoff".
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

static uint16_t crc16Table[256];
static bool crc16TableBuilt = false;

static void
buildCrc16Table (void)
{
	for (uint32_t i = 0; i < 256u; i++)
		{
			uint16_t crc = (uint16_t)(i << 8);
			for (uint32_t bit = 0; bit < 8u; bit++)
				{
					if (crc & 0x8000u)
						{
							crc = (uint16_t)((crc << 1) ^ 0x1021u);
						}
					else
						{
							crc = (uint16_t)(crc << 1);
						}
				}
			crc16Table[i] = crc;
		}
}

void
crc16Init (void)
{
	if (!crc16TableBuilt)
		{
			buildCrc16Table ();
			crc16TableBuilt = true;
		}
}

uint16_t
crc16 (const uint8_t *pBuffer, uint16_t length)
{
	uint16_t crc = 0xFFFFu;

	for (uint16_t i = 0; i < length; i++)
		{
			uint8_t index = (uint8_t)((crc >> 8) ^ pBuffer[i]);
			crc           = (uint16_t)((crc << 8) ^ crc16Table[index]);
		}

	return crc;
}
