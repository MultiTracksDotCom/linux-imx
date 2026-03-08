/*
 * ak4432.c  --  audio driver for AK4432
 *
 * Copyright (C) 2016 Asahi Kasei Microdevices Corporation
 *  Author				Date		Revision	kernel version
 * Samuel Morris    10/7/25		2.0
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Tsuyoshi Mutsuro    16/06/11		1.0			3.18.25
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/regmap.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/firmware.h>
#include <linux/vmalloc.h>
#include <sound/initval.h>
#include <linux/ioctl.h>

#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

#include "ak4432.h"

#ifdef AK4432_DEBUG
#define akdbgprt printk
#else
#define akdbgprt(format, arg...) do {} while (0)
#endif


#define SND_SOC_SPI 1
#define SND_SOC_I2C 2
/* AK4432 Codec Private Data */
struct ak4432_priv {
	int control_type;
#ifdef AK4432_I2C_IF
	struct i2c_client *i2c;
#else
	struct spi_device *spi;
#endif
	int pdn_gpio;
	int i2cfil_gpio;  // I2CFIL pin for I2C mode (not mute in I2C mode)
	int sds;	//SDS1-0 bits
	int digfil;	//DASD,DASW bits
	int fs;		//sampling rate
	unsigned int mclk_freq;	//MCLK frequency
	struct regmap *regmap;
};

static int ak4432_read_register(void *context, unsigned int reg, unsigned int *val);

/* ak4432 register cache & default register settings */
static const struct reg_default ak4432_reg[] = {
	{ AK4432_00_POWER_MANAGEMENT, 0x02 },	/* AK4432_00_POWER_MANAGEMENT - PMDA off */
	{ AK4432_01_CONTROL1, 0x00 },	/* AK4432_01_CONTROL1 - ACKS bit set for auto clock detection */
	{ AK4432_02_DATA_INTERFACE, 0x06 },	/* AK4432_02_DATA_INTERFACE - TDM128, L1 R1 TDM slot, 24-bit LSB justified */
	{ AK4432_03_CONTROL2, 0x01 },	/* AK4432_03_CONTROL2 - Normal operation */
	{ AK4432_04_AOUTL_VOLUME_CONTROL, 0x40 },	/* AK4432_04_AOUTL_VOLUME_CONTROL -20dB */
	{ AK4432_05_AOUTR_VOLUME_CONTROL, 0x40 },	/* AK4432_05_AOUTR_VOLUME_CONTROL -20dB */
};

/* DAC Digital Volume control:
 * from -115.5 to 12 dB in 0.5 dB steps (mute instead of -115.5 dB) */
static DECLARE_TLV_DB_SCALE(latt_tlv, -11550, 50, 0);
static DECLARE_TLV_DB_SCALE(ratt_tlv, -11550, 50, 0);


//DASD, DASL bits Digital Filter Setting
//0,	0 : Sharp Roll-Off Filter
//0,	1 : Slow Roll-Off Filter
//1,	0 : Short delay Sharp Roll-Off Filter
//1,	1 : Short delay Slow Roll-Off Filter
static const char *ak4432_digfil_select_texts[] = {"Sharp Roll-Off Filter", "Slow Roll-Off Filter", "Short delay Sharp Roll-Off Filter", "Short delay Slow Roll-Off Filter"};

//TDM1-0 bits: TDM Mode Setting
// 0 0 : Normal Mode
// 0 1 : TDM128 Mode
// 1 0 : TDM256 Mode
// 1 1 : TDM256 Mode
static const char *ak4432_tdm_select_texts[] = { "Normal Mode", "TDM128 Mode", "TDM256 Mode", "TDM256 Mode"};

//ATS bit Attenuation Speed
static const char* ak4432_ats_select_texts[] = {"4/fs", "16/fs", };
//SDS12-0 bits: Output Data Select
//Refer to Data Sheet
static const char *ak4432_sds_select_texts[] = {
	"L1/R1", "L2/R2", "L3/R3", "L4/R4", };

#ifdef AK4432_DEBUG
static int nReg;
static const char *ak4432_reg_select_texts[] = { "Read AK4432 All Reg", };
static const struct soc_enum ak4432_debug_enum[] =
{
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ak4432_reg_select_texts), ak4432_reg_select_texts),
};

static int get_reg_debug( struct snd_kcontrol *kcontrol,struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = nReg;

	return 0;
}

static int set_reg_debug( struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	u32    reg = ucontrol->value.enumerated.item[0];
	int    i, value;

	nReg= reg ;


	for ( i = AK4432_00_POWER_MANAGEMENT ; i < AK4432_MAX_REGISTERS ; i++ ){
		ak4432_read_register(codec, (u8)i, &value );
		printk("***AK4432 Addr,Reg=(%02X, %02X)\n", i, value);
	}

	return 0;
}

#endif

static int get_digfil(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);	//16/06/13
	struct ak4432_priv *ak4432 = snd_soc_component_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = ak4432->digfil;

	return 0;
};

static int get_sds(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct ak4432_priv *ak4432 = snd_soc_component_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = ak4432->sds;

	return 0;
};

static int set_digfil(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct ak4432_priv *ak4432 = snd_soc_component_get_drvdata(codec);

	int reg, num;
	akdbgprt("\t[AK4432] %s(%d)\n",__FUNCTION__,__LINE__);

	num = ucontrol->value.enumerated.item[0];
	if( num > 4) return -EINVAL;

	ak4432->digfil = num;
	akdbgprt("\t[AK4432] %s(%d) digfil=%d\n",__FUNCTION__,__LINE__, ak4432->digfil);

	//write DASD, DASL bit
	reg = snd_soc_component_read(codec, AK4432_03_CONTROL2);
	reg &= ~AK4432_SDSL_MASK;

	reg |= ((ak4432->digfil & 0x03) << 3);
	snd_soc_component_update_bits( codec, AK4432_03_CONTROL2, AK4432_SDSL_MASK, reg);

	return 0;
};

static int set_sds(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct ak4432_priv *ak4432 = snd_soc_component_get_drvdata(codec);

	int reg;
	akdbgprt("\t[AK4432] %s(%d)\n",__FUNCTION__,__LINE__);

	if(ucontrol->value.enumerated.item[0] > 3) return -EINVAL;

	ak4432->sds = ucontrol->value.enumerated.item[0];

	akdbgprt("\t[AK4432] %s(%d) sds=%d\n",__FUNCTION__,__LINE__, ak4432->sds);

	//write SDS1-0 bits
	reg = snd_soc_component_read(codec, AK4432_02_DATA_INTERFACE);
	reg &= ~AK4432_SDS_MASK;

	reg |= ((ak4432->sds & 0x03) << 5 );
	snd_soc_component_update_bits( codec, AK4432_02_DATA_INTERFACE, AK4432_SDS_MASK, reg);

	return 0;

};


static const struct soc_enum ak4432_dac_enum[] = {
/*0*/	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ak4432_digfil_select_texts),ak4432_digfil_select_texts),
/*1*/	SOC_ENUM_SINGLE(AK4432_03_CONTROL2, 2, ARRAY_SIZE(ak4432_ats_select_texts), ak4432_ats_select_texts),
/* /\*1*\/	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ak4432_sds_select_texts),ak4432_sds_select_texts), */
/* /\*2*\/	SOC_ENUM_SINGLE(AK4432_02_DATA_INTERFACE, 3, ARRAY_SIZE(ak4432_tdm_select_texts), ak4432_tdm_select_texts), */
};

static const struct snd_kcontrol_new ak4432_snd_controls[] = {
	SOC_SINGLE_TLV( "AK4432 Lch Digital Volume", AK4432_04_AOUTL_VOLUME_CONTROL, 0/*shift*/, 0xFF/*max value*/, 1/*invert*/, latt_tlv ),
	SOC_SINGLE_TLV( "AK4432 Rch Digital Volume", AK4432_05_AOUTR_VOLUME_CONTROL, 0, 0xFF, 1, ratt_tlv),

	SOC_ENUM_EXT( "AK4432 Digital Filter Setting", ak4432_dac_enum[0], get_digfil, set_digfil ),
	SOC_ENUM( "AK4432 Attenuation Time Setting", ak4432_dac_enum[1] ),
	/* SOC_ENUM( "AK4432 TDM Mode Setting", ak4432_dac_enum[2] ), */
	/* SOC_ENUM_EXT( "AK4432 SDS Setting", ak4432_dac_enum[1], get_sds, set_sds ), */
/* #ifdef AK4432_DEBUG */
/* 	SOC_ENUM_EXT("All Reg Read", ak4432_debug_enum[0], get_reg_debug, set_reg_debug), */
/* #endif */

};

static const char* ak4432_dac_select_texts[] = {"OFF","ON"};

static const struct soc_enum ak4432_dac_mux_enum =
	SOC_ENUM_SINGLE_VIRT(ARRAY_SIZE(ak4432_dac_select_texts), ak4432_dac_select_texts);
static const struct snd_kcontrol_new ak4432_dac_mux_control =
	SOC_DAPM_ENUM("DAC Switch", ak4432_dac_mux_enum);

/* ak4432 dapm widgets */
static const struct snd_soc_dapm_widget ak4432_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("AK4432 DAC", NULL, AK4432_00_POWER_MANAGEMENT, 2, 0),/*pw*/
	SND_SOC_DAPM_AIF_IN("AK4432 SDTI", "Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("AK4432 AOUT"),

	SND_SOC_DAPM_MUX("DAC to AOUT", SND_SOC_NOPM, 0, 0, &ak4432_dac_mux_control),/*nopm*/
};

static const struct snd_soc_dapm_route ak4432_intercon[] =
{
	{"DAC to AOUT", "ON", "AK4432 SDTI"},
	{"AK4432 DAC", NULL, "DAC to AOUT"},
	{"AK4432 AOUT", NULL, "AK4432 DAC"},
};


static int ak4432_hw_params(
	struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params,
	struct snd_soc_dai *dai)
{
	struct snd_soc_component *codec = dai->component;
	struct ak4432_priv *ak4432 = snd_soc_component_get_drvdata(codec);

#ifdef AK4432_ACKS_USE_MANUAL_MODE
	u8 dfs;
#endif
	int nfs1;
	/* int dif2; */

	akdbgprt("\t[AK4432] %s(%d)\n",__FUNCTION__,__LINE__);
	akdbgprt("\t[AK4432] %s Sample rate: %d, channels: %d, width: %d\n",
		__FUNCTION__, params_rate(params), params_channels(params), params_width(params));

	nfs1 = params_rate(params);
	ak4432->fs = nfs1;

#ifdef AK4432_ACKS_USE_MANUAL_MODE
	dfs = snd_soc_component_read(codec, AK4432_01_CONTROL1);
	dfs &= ~AK4432_DFS_MASK;


	switch (nfs1) {
		case 32000:
		case 44100:
		case 48000:
			dfs |= AK4432_DFS_48KHZ;
			break;
		case 88200:
		case 96000:
			dfs |= AK4432_DFS_96KHZ;
			break;
		case 176400:
		case 192000:
			dfs |= AK4432_DFS_192KHZ;
			break;
		case 384000:
			dfs |= AK4432_DFS_384KHZ;
			break;
		case 768000:
			dfs |= AK4432_DFS_768KHZ;
			break;
		default:
			return -EINVAL;
	}

	snd_soc_component_update_bits(codec, AK4432_01_CONTROL1, AK4432_DFS_MASK,dfs);

#else
	snd_soc_component_update_bits(codec, AK4432_01_CONTROL1,  AK4432_ACKS_MASK, AK4432_ACKS_MASK);
#endif

	return 0;
}

static int ak4432_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
								 unsigned int freq, int dir)
{
	struct snd_soc_component *codec = dai->component;
	struct ak4432_priv *ak4432 = snd_soc_component_get_drvdata(codec);

	akdbgprt("\t[AK4432] %s(%d) clk_id=%d, freq=%u, dir=%d\n",__FUNCTION__,__LINE__, clk_id, freq, dir);

	/* Store the MCLK frequency for use in hw_params */
	ak4432->mclk_freq = freq;

	/* The AK4432 doesn't need explicit MCLK configuration in the driver
	 * as it uses the MCLK directly from the SAI interface */

	return 0;
}

static int ak4432_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *codec = dai->component;
	u8 format;

	/* set master/slave audio interface */
	format = snd_soc_component_read(codec, AK4432_02_DATA_INTERFACE);

	akdbgprt("\t[AK4432] %s(%d) addr 02H = %02X\n",__FUNCTION__,__LINE__, format);
	akdbgprt("\t[AK4432] %s fmt=0x%08x, format_mask=0x%08x\n",__FUNCTION__, fmt, SND_SOC_DAIFMT_FORMAT_MASK);

	format &= ~AK4432_DIF_MASK;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
		case SND_SOC_DAIFMT_CBS_CFS: //Slave Mode
			break;
		case SND_SOC_DAIFMT_CBM_CFM: //Master Mode is not supported by AK4432
		case SND_SOC_DAIFMT_CBS_CFM:
		case SND_SOC_DAIFMT_CBM_CFS:
			default:
			dev_err(codec->dev, "Clock mode unsupported");
			return -EINVAL;
	}

	/* TODO: fill out the entire table with cases for each width */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
		case SND_SOC_DAIFMT_I2S:
			akdbgprt("\t[AK4432] %s using I2S format\n",__FUNCTION__);
			format |= AK4432_DIF_I2S_32B;
			break;
		case SND_SOC_DAIFMT_LEFT_J:
			akdbgprt("\t[AK4432] %s using LEFT_J format\n",__FUNCTION__);
			format |= AK4432_DIF_LSB_32B;
			break;
		case SND_SOC_DAIFMT_DSP_B:
			akdbgprt("\t[AK4432] %s using DSP_B format\n",__FUNCTION__);
			format |= AK4432_DIF_LSB_24B;
			break;
		default:
			akdbgprt("\t[AK4432] %s unsupported format: 0x%08x\n",__FUNCTION__, fmt & SND_SOC_DAIFMT_FORMAT_MASK);
			return -EINVAL;
	}

	/* set format */
	akdbgprt("\t[AK4432] %s(%d) addr 02H = %02X\n",__FUNCTION__,__LINE__, format);
	snd_soc_component_update_bits(codec, AK4432_02_DATA_INTERFACE, AK4432_DIF_MASK,format);

	return 0;
}

static int ak4432_set_tdm_slot(struct snd_soc_dai *dai, unsigned int tx_mask,
			       unsigned int rx_mask, int slots, int slot_width)
{
	struct snd_soc_component *codec = dai->component;

	akdbgprt("\t[AK4432] %s(%d) tx_mask=0x%x, rx_mask=0x%x, slots=%d, slot_width=%d\n",
		__FUNCTION__, __LINE__, tx_mask, rx_mask, slots, slot_width);

	// Validate the TDM configuration
	if (slots != 2) {
		dev_err(codec->dev, "AK4432: Unsupported TDM slots: %d (expected 2)\n", slots);
		return -EINVAL;
	}

	if (slot_width != 32) {
		dev_err(codec->dev, "AK4432: Unsupported TDM slot width: %d (expected 32)\n", slot_width);
		return -EINVAL;
	}

	akdbgprt("\t[AK4432] %s TDM configuration accepted\n", __FUNCTION__);

	return 0;
}

static bool ak4432_writeable(struct device *dev, unsigned int reg)
{
	/* akdbgprt("\t[AK4432] %s reg=%d, max=%d\n",__FUNCTION__, reg, AK4432_MAX_REGISTERS); */
	if (reg < AK4432_MAX_REGISTERS)
		return true;
	else
		return false;
}

static bool ak4432_volatile(struct device *dev, unsigned int reg)
{
	bool	ret;

#ifdef AK4432_IF_DEBUG
	ret = 1;
#else
	ret = 0;
#endif

	return ret;
}

static bool ak4432_readable(struct device *dev, unsigned int reg)
{
	/* akdbgprt("\t[AK4432] %s reg=%d, max=%d\n",__FUNCTION__, reg, AK4432_MAX_REGISTERS); */
	if (reg < AK4432_MAX_REGISTERS)
		return true;
	else
		return false;
}


static int ak4432_read_register(void *context, unsigned int reg, unsigned int *val)
{
	struct ak4432_priv *ak4432 = context;
	akdbgprt("\t[AK4432] %s ENTRY reg=%d\n",__FUNCTION__, reg);
#ifdef AK4432_I2C_IF
	struct i2c_msg xfer[2];
#endif
	u8	tx[3], rx[1];
	int	wlen, rlen, ret;
	wlen = 3;
	rlen = 1;

	tx[0] = (u8)AK4432_COMMAND_CODE_READ;
	tx[1] = (u8)0x00;
	tx[2] = (u8)(reg&0x07);

#ifdef AK4432_I2C_IF //I2C
	/* Write Register*/
	xfer[0].addr = ak4432->i2c->addr;
	xfer[0].flags = 0;
	xfer[0].len = wlen;
	xfer[0].buf = tx;

	/* Read Register*/
	xfer[1].addr = ak4432->i2c->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = rlen;
	xfer[1].buf = rx;

	ret = i2c_transfer(ak4432->i2c->adapter, xfer, 2 );

#else	//SPI

	ret = spi_write_then_read( ak4432->spi, tx, wlen, rx, rlen );

#endif
	*val = rx[0];
	akdbgprt("\t[AK4432] %s reg=%d, val=0x%02x\n",__FUNCTION__, reg, *val);
	return ret;
}

static int ak4432_write_register(void *context, unsigned int reg, unsigned int val)
{
	struct ak4432_priv *ak4432 = context;
	akdbgprt("\t[AK4432] %s ENTRY reg=%d, val=0x%02x\n",__FUNCTION__, reg, val);
	unsigned char tx[4];
	int wlen;
	int rc;

	akdbgprt("\t[AK4432] %s (%02x, %02x) called from regmap\n",__FUNCTION__, reg, val);

	//tx[0] ...CommandCode[C0(write) or 40(Read)]
	//tx[1] ...HI byte of address(fixed to 00)
	//tx[2] ...LO byte of address(D2-D0 are valid)
	//tx[3] ...Data

	wlen = 4;
	tx[0] = (u8)AK4432_COMMAND_CODE_WRITE;
	tx[1] = (u8)0x00;
	tx[2] = (u8)(reg & 0x07);	//register address
	tx[3] = val;	//8bit

#ifdef AK4432_I2C_IF
	rc = i2c_master_send( ak4432->i2c, tx, wlen );
#else
	rc = spi_write(ak4432->spi, tx, wlen);

#endif

	return rc;
}


// * for AK4432
static int ak4432_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *codec_dai)
{
	struct snd_soc_component *codec = codec_dai->component;
	struct ak4432_priv *ak4432 = snd_soc_component_get_drvdata(codec);
	int ret = 0;

	akdbgprt("\t[AK4432] %s(%d) cmd=%d\n",__FUNCTION__,__LINE__, cmd);

	/* The trigger function is called in atomic context, so we cannot perform
	 * I2C operations here. The mute/unmute functionality should be handled
	 * by the mute_stream function instead, which is called from a safe context.
	 * For now, we'll just log the trigger events without doing I2C operations.
	 */

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		akdbgprt("\t[AK4432] %s Playback started\n",__FUNCTION__);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		akdbgprt("\t[AK4432] %s Playback stopped\n",__FUNCTION__);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}


static int ak4432_set_bias_level(struct snd_soc_component *codec,
								 enum snd_soc_bias_level level)
{
	akdbgprt("\t[AK4432] %s(%d) level=%d\n",__FUNCTION__,__LINE__, level);

	switch (level) {
		case SND_SOC_BIAS_ON:
			// Enable DAC when bias is ON
			snd_soc_component_update_bits(codec, AK4432_00_POWER_MANAGEMENT, AK4432_PMDA_MASK, AK4432_PMDA_MASK);
			break;
		case SND_SOC_BIAS_PREPARE:
			// Keep DAC enabled during prepare
			snd_soc_component_update_bits(codec, AK4432_00_POWER_MANAGEMENT, AK4432_PMDA_MASK, AK4432_PMDA_MASK);
			break;
		case SND_SOC_BIAS_STANDBY:
			// Keep DAC enabled in standby
			snd_soc_component_update_bits(codec, AK4432_00_POWER_MANAGEMENT, AK4432_PMDA_MASK, AK4432_PMDA_MASK);
			break;
		case SND_SOC_BIAS_OFF:
			// Disable DAC when bias is OFF
			snd_soc_component_update_bits(codec, AK4432_00_POWER_MANAGEMENT, AK4432_PMDA_MASK, 0);
			break;
	}
	codec->dapm.bias_level = level;

	return 0;
}

static int ak4432_set_dai_mute(struct snd_soc_dai *dai, int mute, int direction)
{
	struct snd_soc_component *codec = dai->component;
	struct ak4432_priv *ak4432 = snd_soc_component_get_drvdata(codec);
	int nfs, ndt, ret, reg, att_step;
	int ats;	//ATS bits

	nfs = ak4432->fs;

	reg = snd_soc_component_read( codec,  AK4432_03_CONTROL2);
	ats = ( reg & AK4432_ATS_MASK ) >> 2;

	akdbgprt("\t[AK4432] %s mute[%s] nfs[%d] direction[%d]\n", __FUNCTION__,
		 mute ? "ON" : "OFF", nfs, direction);

	akdbgprt("\t[AK4432] %s direction value: %d\n", __FUNCTION__, direction);

	switch( ats ) {
		case 0:
			att_step = 1021;	break;
		case 1:
			att_step = 4084;	break;
	}

	ndt = att_step / (nfs / 1000);

	if (mute) {	//SMUTE: 1 , MUTE
		ret = snd_soc_component_update_bits(codec, AK4432_03_CONTROL2, AK4432_SMUTE_MASK, AK4432_SMUTE_MASK); //softmute
		mdelay(ndt);
		akdbgprt("\t[AK4432] %s(%d) mdelay(%d ms)\n",__FUNCTION__,__LINE__, ndt);
		akdbgprt("\t[AK4432] %s External Mute = ON\n",__FUNCTION__);
	}
	else {		// SMUTE: 0 ,NORMAL operation
		akdbgprt("\t[AK4432] %s External Mute = OFF\n",__FUNCTION__);
		ret = snd_soc_component_update_bits(codec, AK4432_03_CONTROL2, AK4432_SMUTE_MASK, 0);
		mdelay(ndt);
		akdbgprt("\t[AK4432] %s(%d) mdelay(%d ms)\n",__FUNCTION__,__LINE__, ndt);
	}
	akdbgprt("\t[AK4432] %s(%d) ret[%d]\n",__FUNCTION__,__LINE__, ret);

	return 0;
}

#define AK4432_RATES		(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
	SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
	SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
	SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |\
	SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |\
	SNDRV_PCM_RATE_192000)  // | SNDRV_PCM_RATE_384000 | SNDRV_PCM_RATE_768000

#define AK4432_FORMATS		SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE


static struct snd_soc_dai_ops ak4432_dai_ops = {
	.hw_params	= ak4432_hw_params,
	.set_sysclk	= ak4432_set_dai_sysclk,
	/* .set_fmt	= ak4432_set_dai_fmt, */
	/* .set_tdm_slot	= ak4432_set_tdm_slot, */
	.trigger = ak4432_trigger,
	/* .mute_stream = ak4432_set_dai_mute, */
};

struct snd_soc_dai_driver ak4432_dai[] = {
	{
		.name = "ak4432-aif",
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = AK4432_RATES,
			.formats = AK4432_FORMATS,
		},
		.ops = &ak4432_dai_ops,
	},
};

static int ak4432_init_reg(struct snd_soc_component *codec)
{
	struct ak4432_priv *ak4432 = snd_soc_component_get_drvdata(codec);
	int ret = 0;

	akdbgprt("\t[AK4432] %s regmap=%p\n",__FUNCTION__, ak4432->regmap);
	akdbgprt("\t[AK4432] %s component->regmap=%p\n",__FUNCTION__, codec->regmap);

	// Configure I2CFIL GPIO for proper I2C operation
	// Based on working GPIO hog configuration, I2CFIL should be LOW
	if ( ak4432->i2cfil_gpio > 0 ) gpio_set_value(ak4432->i2cfil_gpio, 0); // I2CFIL LOW for I2C mode

	/* TODO: we should be using a regulator here */
	if ( ak4432->pdn_gpio > 0 ) {
		gpio_set_value(ak4432->pdn_gpio, 0);
		msleep(1);
		gpio_set_value(ak4432->pdn_gpio, 1);
		msleep(1);
	}

	// Turn off power during initialization
	snd_soc_component_update_bits(codec, AK4432_00_POWER_MANAGEMENT, AK4432_PMDA_MASK, 0);
	// ACKS bit set for auto clock detection
	snd_soc_component_update_bits(codec, AK4432_01_CONTROL1, AK4432_ACKS_MASK, AK4432_ACKS_MASK);
	snd_soc_component_update_bits(codec, AK4432_01_CONTROL1, AK4432_DFS_MASK, AK4432_DFS_48KHZ);
	snd_soc_component_update_bits(codec, AK4432_02_DATA_INTERFACE, AK4432_DIF_MASK, AK4432_DIF_LSB_24B);
	snd_soc_component_update_bits(codec, AK4432_02_DATA_INTERFACE, AK4432_TDM_MASK, 0x10);
	snd_soc_component_update_bits(codec, AK4432_02_DATA_INTERFACE, AK4432_SDS_MASK, 0);
	snd_soc_component_update_bits(codec, AK4432_03_CONTROL2, AK4432_SYNCE_MASK, 0);
	snd_soc_component_update_bits(codec, AK4432_03_CONTROL2, AK4432_SMUTE_MASK, 0);
	snd_soc_component_update_bits(codec, AK4432_03_CONTROL2, AK4432_ATS_MASK, 0);
	snd_soc_component_update_bits(codec, AK4432_04_AOUTL_VOLUME_CONTROL, 0xFF, 0x40);
	snd_soc_component_update_bits(codec, AK4432_05_AOUTR_VOLUME_CONTROL, 0xFF, 0x40);
	snd_soc_component_update_bits(codec, AK4432_00_POWER_MANAGEMENT, AK4432_PMDA_MASK, AK4432_PMDA_MASK);

	akdbgprt("\t[AK4432] %s(%d)\n",__FUNCTION__,__LINE__);
	return 0;
}

#ifdef CONFIG_OF
static int ak4432_parse_dt(struct ak4432_priv *ak4432)
{
	struct device *dev;
	struct device_node *np;

	dev = NULL;

	if (ak4432->control_type == SND_SOC_SPI) {
#ifndef AK4432_I2C_IF
		dev = &(ak4432->spi->dev);
#endif
	}
	else {
#ifdef AK4432_I2C_IF
		dev = &(ak4432->i2c->dev);
#endif
	}

	np = dev->of_node;

	if (!np)
		return -1;

	printk("Read PDN pin from device tree\n");

	ak4432->pdn_gpio = of_get_named_gpio(np, "ak4432,pdn-gpio", 0);
	if (ak4432->pdn_gpio < 0) {
		printk(KERN_ERR "ak4432 pdn pin(%u) is invalid(%d)\n", ak4432->pdn_gpio, __LINE__);
		ak4432->pdn_gpio = -1;
		return -1;
	}

	if( !gpio_is_valid(ak4432->pdn_gpio) ) {
		printk(KERN_ERR "ak4432 pdn pin(%u) is invalid\n", ak4432->pdn_gpio);
		return -1;
	}

	printk("Read i2cfil pin from device tree\n");

	ak4432->i2cfil_gpio = of_get_named_gpio(np, "ak4432,i2cfil-gpio", 0);
	if (ak4432->i2cfil_gpio < 0) {
		printk(KERN_ERR "ak4432 i2cfil pin(%u) is invalid(%d)\n", ak4432->i2cfil_gpio, __LINE__);
		ak4432->i2cfil_gpio = -1;
		return -1;
	}

	if( !gpio_is_valid(ak4432->i2cfil_gpio) ) {
		printk(KERN_ERR "ak4432 i2cfil_gpio(%u) is invalid\n", ak4432->i2cfil_gpio);
		return -1;
	}

	return 0;
}
#endif

static int ak4432_probe(struct snd_soc_component *codec)
{
	struct ak4432_priv *ak4432 = snd_soc_component_get_drvdata(codec);
	int ret = 0;

	akdbgprt("\t[AK4432] %s(%d)\n",__FUNCTION__,__LINE__);


	/* Configure GPIOs BEFORE reg init */
	ret = ak4432_parse_dt(ak4432);
	if( ret < 0 ) {
		ak4432->pdn_gpio = -1;
		ak4432->i2cfil_gpio = -1;
		ret = 0;
	}
	if ( ak4432->i2cfil_gpio > 0 ) {
		ret = gpio_request(ak4432->i2cfil_gpio, "ak4432 i2cfil");
		gpio_direction_output(ak4432->i2cfil_gpio, 0);
		gpio_set_value(ak4432->i2cfil_gpio, 0);
		akdbgprt("\t[AK4432] %s I2CFIL GPIO %d configured LOW\n",__FUNCTION__, ak4432->i2cfil_gpio);
	}

	if ( ak4432->pdn_gpio > 0 ) {
		ret = gpio_request(ak4432->pdn_gpio, "ak4432 pdn");
		gpio_direction_output(ak4432->pdn_gpio, 0);
		akdbgprt("\t[AK4432] %s PDN GPIO %d configured\n",__FUNCTION__, ak4432->pdn_gpio);
	}

	ak4432_init_reg(codec);

	ak4432->fs = 48000;
	ak4432->sds = 0;
	ak4432->digfil = 0;

	akdbgprt("\t[AK4432] %s(%d) return %d\n",__FUNCTION__,__LINE__, ret);

	return ret;
}

static void ak4432_remove(struct snd_soc_component *codec)
{
	struct ak4432_priv *ak4432 = snd_soc_component_get_drvdata(codec);

	akdbgprt("\t[AK4432] %s(%d)\n",__FUNCTION__,__LINE__);

	ak4432_set_bias_level(codec, SND_SOC_BIAS_OFF);
	if ( ak4432->pdn_gpio > 0 ) {
		gpio_set_value(ak4432->pdn_gpio, 0);
		gpio_free(ak4432->pdn_gpio);
	}
	if ( ak4432->i2cfil_gpio > 0 ) {
		gpio_free(ak4432->i2cfil_gpio);
	}
}

static int ak4432_suspend(struct snd_soc_component *codec)
{
	struct ak4432_priv *ak4432 = snd_soc_component_get_drvdata(codec);

	akdbgprt("\t[AK4432] %s(%d)\n",__FUNCTION__,__LINE__);

	ak4432_set_bias_level(codec, SND_SOC_BIAS_OFF);
	if ( ak4432->pdn_gpio > 0 ) {
		gpio_set_value(ak4432->pdn_gpio, 0);
		snd_soc_component_cache_sync(codec);
	}
	else {
#ifdef AK4432_PD_SUSPEND
		snd_soc_component_cache_sync(codec);
#endif
	}

	return 0;
}

static int ak4432_resume(struct snd_soc_component *codec)
{
	ak4432_init_reg(codec);

	return 0;
}


struct snd_soc_component_driver soc_codec_dev_ak4432 = {
	.probe = ak4432_probe,
	.remove = ak4432_remove,
	.suspend = ak4432_suspend,
	.resume = ak4432_resume,

	.controls = ak4432_snd_controls,
	.num_controls = ARRAY_SIZE(ak4432_snd_controls),
	.set_bias_level = ak4432_set_bias_level,
	.idle_bias_on = 1,
	/* .dapm_widgets = ak4432_dapm_widgets, */
	/* .num_dapm_widgets = ARRAY_SIZE(ak4432_dapm_widgets), */
	/* .dapm_routes = ak4432_intercon, */
	/* .num_dapm_routes = ARRAY_SIZE(ak4432_intercon), */
	/* .use_pmdown_time = 1, */
	.endianness = 1,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_ak4432);

static const struct regmap_config ak4432_regmap = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = AK4432_MAX_REGISTERS,
	.volatile_reg = ak4432_volatile,
	.writeable_reg = ak4432_writeable,
	.readable_reg = ak4432_readable,

	.reg_defaults = ak4432_reg,
	.num_reg_defaults = ARRAY_SIZE(ak4432_reg),
	.cache_type = REGCACHE_FLAT,
	.reg_read = ak4432_read_register,
	.reg_write = ak4432_write_register,
};

#ifdef CONFIG_OF
static struct of_device_id ak4432_if_dt_ids[] = {
	{ .compatible = "akm,ak4432"},
    { }
};
MODULE_DEVICE_TABLE(of, ak4432_if_dt_ids);
#endif

#ifdef AK4432_I2C_IF

static int ak4432_i2c_probe(struct i2c_client *i2c)
{
	struct ak4432_priv *ak4432;
	int ret = 0;

	akdbgprt("\t[AK4432] %s(%d)\n",__FUNCTION__,__LINE__);

	ak4432 = kzalloc(sizeof(struct ak4432_priv), GFP_KERNEL);
	if( ak4432 == NULL) return -ENOMEM;

	i2c_set_clientdata(i2c, ak4432);

	ak4432->control_type = SND_SOC_I2C;
	ak4432->i2c = i2c;

	akdbgprt("\t[AK4432] %s about to create regmap with config: reg_read=%p, reg_write=%p\n",__FUNCTION__,
		ak4432_regmap.reg_read, ak4432_regmap.reg_write);
	ak4432->regmap = devm_regmap_init(&i2c->dev, NULL, ak4432, &ak4432_regmap);
	if(IS_ERR(ak4432->regmap)) {
		akdbgprt("\t[AK4432] %s regmap creation failed: %ld\n",__FUNCTION__, PTR_ERR(ak4432->regmap));
		return PTR_ERR(ak4432->regmap);
	}

	akdbgprt("\t[AK4432] %s regmap created: %p\n",__FUNCTION__, ak4432->regmap);

	dev_set_drvdata(&i2c->dev, ak4432);

	ret = devm_snd_soc_register_component(&i2c->dev,
		&soc_codec_dev_ak4432, &ak4432_dai[0], ARRAY_SIZE(ak4432_dai));
	if(ret < 0 ){
		kfree(ak4432);
		akdbgprt("\t[AK4432 Error!] %s(%d)\n",__FUNCTION__,__LINE__);
	}
	akdbgprt("\t[AK4432] %s(%d)\n",__FUNCTION__,__LINE__);

	return ret;
}

static void ak4432_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_component(&client->dev);
	kfree(i2c_get_clientdata(client));
}

static const struct i2c_device_id ak4432_i2c_id[] = {
	{ "ak4432", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ak4432_i2c_id);

static struct i2c_driver ak4432_i2c_driver = {
	.driver = {
		.name = "ak4432",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(ak4432_if_dt_ids),
#endif
	},
	.probe = ak4432_i2c_probe,
	.remove = ak4432_i2c_remove,
	.id_table = ak4432_i2c_id,
};

#else

static int ak4432_spi_probe(struct spi_device *spi)
{
	struct ak4432_priv *ak4432;
	int	ret;

	akdbgprt("\t[AK4432] %s spi=%x\n",__FUNCTION__, (int)spi);

	ak4432 = kzalloc(sizeof(struct ak4432_priv), GFP_KERNEL);
	if (!ak4432)
		return -ENOMEM;

	ak4432->control_type = SND_SOC_SPI;
	ak4432->spi = spi;

	ak4432->regmap = devm_regmap_init(&spi->dev, NULL, ak4432, &ak4432_regmap);
	if(IS_ERR(ak4432->regmap))
		return PTR_ERR(ak4432->regmap);

	spi_set_drvdata(spi, ak4432);
	dev_set_drvdata(&spi->dev, ak4432);

	ret = devm_snd_soc_register_component(&spi->dev,
		&soc_codec_dev_ak4432,  &ak4432_dai[0], ARRAY_SIZE(ak4432_dai));
	akdbgprt("\t[AK4432] %s ret=%d\n",__FUNCTION__, ret);
	if (ret < 0) {
		kfree(ak4432);
		akdbgprt("\t[AK4432 Error!] %s(%d)\n",__FUNCTION__,__LINE__);
	}


	return 0;
}

static int ak4432_spi_remove(struct spi_device *spi)
{
	kfree(spi_get_drvdata(spi));
	return 0;
}

static struct spi_driver ak4432_spi_driver = {
	.driver = {
		.name = "ak4432",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(ak4432_if_dt_ids),
#endif
	},
	.probe = ak4432_spi_probe,
	.remove = ak4432_spi_remove,
};
#endif

static int __init ak4432_modinit(void)
{
	int ret = 0;

	akdbgprt("\t[AK4432] %s(%d)\n", __FUNCTION__,__LINE__);

#ifdef AK4432_I2C_IF
	akdbgprt("\t[AK4432] %s(%d)\n", __FUNCTION__,__LINE__);
	ret = i2c_add_driver(&ak4432_i2c_driver);
	if( ret != 0 ){
		printk(KERN_ERR "Faild to register AK4432 I2C driver: %d\n", ret);
	}
#else
	ret = spi_register_driver(&ak4432_spi_driver);
	if ( ret != 0 ) {
		printk(KERN_ERR "Failed to register AK4432 SPI driver: %d\n",  ret);
	}
#endif

	akdbgprt("\t[AK4432] %s(%d) return %d\n", __FUNCTION__,__LINE__, ret);
	return ret;
}

module_init(ak4432_modinit);

static void __exit ak4432_exit(void)
{
#ifdef AK4432_I2C_IF
	i2c_del_driver(&ak4432_i2c_driver);
#else
	spi_unregister_driver(&ak4432_spi_driver);
#endif
}
module_exit(ak4432_exit);

MODULE_AUTHOR("Samuel Morris <scmorris.dev@gmail.com>");
MODULE_DESCRIPTION("ASoC ak4432 DAC driver");
MODULE_LICENSE("GPL");
