//******************************************************************************
// @file      : spi_transport_hw.c
// @brief     : Generic (adapter-independent) setter for the HW-adapter's
//              core-side callback fields. See spi_transport_hw.h.
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

#include "spi_transport/spi_transport_hw.h"

void
spiTransportHwSetCallbacks (trSpiTransportHw *prHw,
                            void (*pOnTransferComplete) (void *pCoreCtx, uint16_t length),
                            void (*pOnSelectEvent) (void *pCoreCtx, bool asserted),
                            void (*pOnReadyEvent) (void *pCoreCtx, bool asserted),
                            void (*pOnClockStart) (void *pCoreCtx), void *pCoreCtx)
{
	prHw->pOnTransferComplete = pOnTransferComplete;
	prHw->pOnSelectEvent      = pOnSelectEvent;
	prHw->pOnReadyEvent       = pOnReadyEvent;
	prHw->pOnClockStart       = pOnClockStart;
	prHw->pCoreCtx            = pCoreCtx;
}
