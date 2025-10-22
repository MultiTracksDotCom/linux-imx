/*
 * ak4432.h  --  audio driver for AK4432
 *
 * Copyright (C) 2016 Asahi Kasei Microdevices Corporation
 *  Author				Date		Revision	kernel version
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Tsuyoshi Mutsuro    16/06/11		1.0			3.0.31
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/

#ifndef _AK4432_H
#define _AK4432_H

#include <linux/bits.h>

/***************************************************
				  Settings
****************************************************/
#define AK4432_I2C_IF			//I2C IF
//#define AK4432_PD_SUSPEND		
//#define AK4432_DEBUG			//used at debug mode
//#define AK4432_ACKS_USE_MANUAL_MODE

#define AK4432_COMMAND_CODE_WRITE	0xC0
#define AK4432_COMMAND_CODE_READ	0x40


#define AK4432_00_POWER_MANAGEMENT	0x00
#define AK4432_PMDA_MASK		0x02

#define AK4432_01_CONTROL1		0x01
#define AK4432_ACKS_MASK		0x01
#define AK4432_DFS_MASK		0x06

#define AK4432_02_DATA_INTERFACE	0x02
#define AK4432_DIF_MASK		0x07
#define AK4432_TDM_MASK		0x18
#define AK4432_SDS_MASK		0x60

#define AK4432_03_CONTROL2		0x03
#define AK4432_SYNCE_MASK		0x01
#define AK4432_SMUTE_MASK		0x02
#define AK4432_ATS_MASK		0x04

#define AK4432_04_AOUTL_VOLUME_CONTROL	0x04
#define AK4432_05_AOUTR_VOLUME_CONTROL	0x05


#define AK4432_MAX_REGISTERS	(AK4432_05_AOUTR_VOLUME_CONTROL+ 1)

//Digital Filter (DASL, DASD)
#define AK4432_SDSL_MASK		0x18
//---------------------------------------------------------------------------

//DIF2 1 0
#define AK4432_DIF2_SHIFT		2
#define AK4432_DIF_LSB_16B	(0)
#define AK4432_DIF_LSB_20B	(1)
#define AK4432_DIF_MSB_24B	(2)
#define AK4432_DIF_I2S_24B	(3)
#define AK4432_DIF_LSB_24B	(4)
#define AK4432_DIF_LSB_32B	(5)
#define AK4432_DIF_MSB_32B	(6)
#define AK4432_DIF_I2S_32B	(7)

#define AK4432_DFS_48KHZ		(0x0 << 1)  //  30kHz to 54kHz
#define AK4432_DFS_96KHZ		(0x1 << 1)  //  54kHz to 108kHz
#define AK4432_DFS_192KHZ		(0x2 << 1)  //  120kHz  to 216kHz
#define AK4432_DFS_384KHZ		(0x0 << 1)	//	384kHz
#define AK4432_DFS_768KHZ		(0x1 << 1)	//	768kHz

#endif
