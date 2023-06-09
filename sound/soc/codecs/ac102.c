/*
 * * (C) Copyright 2017-2018
 * ac102.c --  ac102 ALSA Soc Audio driver
 *
 * Version: 1.0
 *
 * Author: panjunwen
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <linux/of.h>
#include <sound/tlv.h>
#include <linux/regulator/consumer.h>
#include <linux/io.h>

#include "ac102.h"


#define AC102_DEBUG_EN			1

#if AC102_DEBUG_EN
#define AC102_DEBUG(...)		printk(__VA_ARGS__)
#else
#define AC102_DEBUG(...)
#endif


//test config
#define AC102_ADC_PATTERN_SEL	ADC_PTN_NORMAL		//0:ADC normal,  1:0x5A5A5A,  2:0x123456,  3:ADC_PTN_RX_MIX_DATA
#define AC102_DAC_PATTERN_SEL	DAC_PTN_NORMAL		//0:DAC normal,  1:-6dB sin,    2:-60dB sin,  3:zero data


//AC102 config
#define AC102_CHIP_NUMS			1					//range[1, 4]
#define AC102_SLOT_WIDTH		16					//8/12/16/20/24/28/32bit Slot Width
//#define AC102_LRCK_PERIOD		(AC102_SLOT_WIDTH*(AC102_CHIP_NUMS<2? 2: AC102_CHIP_NUMS)/2)	//range[1, 1024], default PCM mode, I2S/LJ/RJ mode shall divide by 2
#define AC102_LRCK_PERIOD		32
//#define AC102_LRCK_PERIOD (AC102_SLOT_WIDTH*(AC102_CHIP_NUMS<2?2: AC102_CHIP_NUMS)/2)
#define AC102_MATCH_DTS_EN		0					//AC102 match method select: 0: i2c_detect, 1:devices tree

#define AC102_ADC_CLK_DIV		ADC_DAC_CLK_DIV_4	//AC102 ADC CLK divide ratio
#define AC102_DAC_CLK_DIV		ADC_DAC_CLK_DIV_2	//AC102 DAC CLK divide ratio
#define AC102_AGC_EN			0					//AC102 AGC enable
#define AC102_EQ_EN				0					//AC102 EQ enable

#define AC102_DAPM_EN			1
#define AC102_CODEC_RW_USER_EN	1
#define AC102_PGA_GAIN			ADC_PGA_GAIN_0dB			//0~31dB,1dB step
#define AC102_LINEOUT_GAIN		DAC_LINEOUT_GAIN_MINUS_21dB	//-45~0dB,3dB step
#define AC102_DMIC_EN			0							//0:ADC	 1:DMIC


#define AC102_REGULATOR_NAME	"ac102_vcc_3v3"
#define AC102_RATES 			(SNDRV_PCM_RATE_8000_48000 | SNDRV_PCM_RATE_KNOT)
#define AC102_FORMATS			(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)


struct i2c_client *i2c_ctrl[AC102_CHIP_NUMS];
int	regulator_en;

struct voltage_supply {
	struct regulator *vcc;
};

struct ac102_priv {
	struct i2c_client *i2c;
	struct snd_soc_codec *codec;
	struct voltage_supply vol_supply;
};

static const struct regmap_config ac102_regmap_config = {
	.reg_bits = 8,	//Number of bits in a register address
	.val_bits = 8,	//Number of bits in a register value
};


struct real_val_to_reg_val {
	unsigned int real_val;
	u8 reg_val;
};



static const struct real_val_to_reg_val ac102_bclk_div[] = {
	{0,  0},
	{1,  1},
	{2,  2},
	{4,  3},
	{6,  4},
	{8,  5},
	{12, 6},
	{16, 7},
	{24, 8},
	{32, 9},
	{48, 10},
	{64, 11},
	{96, 12},
	{128,13},
	{176,14},
	{192,15},
};


static const DECLARE_TLV_DB_SCALE(adc_pga_gain_tlv,0,100,0);
static const DECLARE_TLV_DB_SCALE(adc_dig_vol_tlv,-6450,50,0);
static const DECLARE_TLV_DB_SCALE(dac_lineout_gain_tlv,-4500,300,0);
static const DECLARE_TLV_DB_SCALE(dac_dig_vol_tlv,-6450,50,0);
static const DECLARE_TLV_DB_SCALE(mixer_gain_tlv,-600,600,0);
static const DECLARE_TLV_DB_SCALE(agc_hysteresis_tlv,100,100,0);
static const DECLARE_TLV_DB_SCALE(agc_target_level_tlv,-3000,100,0);
static const DECLARE_TLV_DB_SCALE(agc_max_gain_tlv,0,50,0);
static const DECLARE_TLV_DB_SCALE(agc_noise_threshold_tlv,-3000,200,0);
static const DECLARE_TLV_DB_SCALE(agc_gain_hysteresis_tlv,0,100,0);


/*************************************** General(volume) controls *******************************************/
//ac102 common controls
static const struct snd_kcontrol_new ac102_controls[] = {
	SOC_SINGLE_TLV("ADC PGA Gain", ADC_ANA_CTRL1, PGA_GAIN_CTRL, 0x1F, 0, adc_pga_gain_tlv),
	SOC_SINGLE_TLV("ADC Digital Vol", ADC_DVC, 0, 0xFF, 0, adc_dig_vol_tlv),
	SOC_SINGLE_TLV("DAC LineOut Gain", DAC_ANA_CTRL2, LINE_OUT_AMP_GAIN, 0x0F, 1, dac_lineout_gain_tlv),
	SOC_SINGLE_TLV("DAC Digital Vol", DAC_DVC, 0, 0xFF, 0, dac_dig_vol_tlv),

	SOC_SINGLE_TLV("TX Mixer_L RECD_DAT Gain", I2S_TX_MIX_SRC, TX_MIXL_RECD_GAIN, 0x1, 0, mixer_gain_tlv),
	SOC_SINGLE_TLV("TX Mixer_L PLAY_DAT Gain", I2S_TX_MIX_SRC, TX_MIXL_PLAY_GAIN, 0x1, 0, mixer_gain_tlv),
	SOC_SINGLE_TLV("TX Mixer_R RECD_DAT Gain", I2S_TX_MIX_SRC, TX_MIXR_RECD_GAIN, 0x1, 0, mixer_gain_tlv),
	SOC_SINGLE_TLV("TX Mixer_R RXM_DAT Gain",  I2S_TX_MIX_SRC, TX_MIXR_RXM_GAIN,  0x1, 0, mixer_gain_tlv),

	SOC_SINGLE_TLV("RX Mixer RXL Gain", I2S_RX_MIX_SRC, RX_MIX_RXL_GAIN, 0x1, 0, mixer_gain_tlv),
	SOC_SINGLE_TLV("RX Mixer RXR Gain", I2S_RX_MIX_SRC, RX_MIX_RXR_GAIN, 0x1, 0, mixer_gain_tlv),

	SOC_SINGLE_TLV("DAC Mixer RXM_DAT Gain",  DAC_MIX_SRC, DAC_MIX_RXM_GAIN,  0x1, 0, mixer_gain_tlv),
	SOC_SINGLE_TLV("DAC Mixer RECD_DAT Gain", DAC_MIX_SRC, DAC_MIX_RECD_GAIN, 0x1, 0, mixer_gain_tlv),

	SOC_SINGLE_TLV("AGC Max Gain", AGC_MAXG, 0, 0x50, 0, agc_max_gain_tlv),
	SOC_SINGLE_TLV("AGC Hysteresis", AGC_CTRL, AGC_HYS_SET, 0x3, 0, agc_hysteresis_tlv),
	SOC_SINGLE_TLV("AGC Gain Hysteresis", AGC_OPT, AGC_GAIN_HYS_SET, 0x3, 0, agc_gain_hysteresis_tlv),
	SOC_SINGLE_TLV("AGC Noise Threshold", AGC_NTH, AGC_NOISE_THRES, 0x1F, 0, agc_noise_threshold_tlv),
	SOC_SINGLE_RANGE_TLV("AGC Target Level", AGC_TGLVL, AGC_TGLVL_SET, 0x22, 0x3f, 0, agc_target_level_tlv),

	//debug control
	SOC_SINGLE("ADC HPF Switch", AGC_CTRL, HPF_EN, 0x1, 0),
	SOC_SINGLE("ADC Pattern Sel", ADC_DIG_CTRL, ADC_PTN_SEL, 0x03, 0),
	SOC_SINGLE("DAC Pattern Sel", DAC_DIG_CTRL, DAC_PTN_SEL, 0x03, 0),
};


/*************************************** DAPM controls *******************************************/
//ADC DMIC Source Select Mux
static const char *adc_dmic_src_mux_text[] = {
	"ADC switch", "DMIC switch"
};
static const struct soc_enum adc_dmic_src_mux_enum =
	SOC_ENUM_SINGLE(ADC_DIG_CTRL, DIG_MIC_EN, 2, adc_dmic_src_mux_text);
static const struct snd_kcontrol_new adc_dmic_src_mux =
	SOC_DAPM_ENUM("ADC DMIC MUX", adc_dmic_src_mux_enum);

//TX Left Mixer source control
static const struct snd_kcontrol_new tx_left_src_mixer[] = {
	SOC_DAPM_SINGLE("RECD_DAT switch", I2S_TX_MIX_SRC, TX_MIXL_RECD_SRC, 1, 0),
	SOC_DAPM_SINGLE("PLAY_DAT switch", I2S_TX_MIX_SRC, TX_MIXL_PLAY_SRC, 1, 0),
};

//TX Right Mixer source control
static const struct snd_kcontrol_new tx_right_src_mixer[] = {
	SOC_DAPM_SINGLE("RECD_DAT switch", I2S_TX_MIX_SRC, TX_MIXR_RECD_SRC, 1, 0),
	SOC_DAPM_SINGLE("RXM switch", I2S_TX_MIX_SRC, TX_MIXR_RXM_SRC, 1, 0),
};

//RX Mixer source control
static const struct snd_kcontrol_new rx_src_mixer[] = {
	SOC_DAPM_SINGLE("RXL switch", I2S_RX_MIX_SRC, RX_MIX_RXL_SRC, 1, 0),
	SOC_DAPM_SINGLE("RXR switch", I2S_RX_MIX_SRC, RX_MIX_RXR_SRC, 1, 0),
};

//DAC Mixer source control
static const struct snd_kcontrol_new dac_src_mixer[] = {
	SOC_DAPM_SINGLE("RXM switch", DAC_MIX_SRC, DAC_MIX_RXM_SRC, 1, 0),
	SOC_DAPM_SINGLE("RECD_DAT switch", DAC_MIX_SRC, DAC_MIX_RECD_SRC, 1, 0),
};


/*************************************** DAPM widgets *******************************************/
//ac102 dapm widgets
static const struct snd_soc_dapm_widget ac102_dapm_widgets[] = {
	//input widgets
	SND_SOC_DAPM_INPUT("MICP"),
	SND_SOC_DAPM_INPUT("MICN"),
	SND_SOC_DAPM_INPUT("DMIC"),

	//output widgets
	SND_SOC_DAPM_OUTPUT("LINEOUTP"),
	SND_SOC_DAPM_OUTPUT("LINEOUTN"),

	//MIC PGA
	SND_SOC_DAPM_PGA("MIC PGA", ADC_ANA_CTRL1, ADC_PGA_GEN, 0, NULL, 0),

	//ADC DMIC MUX
	SND_SOC_DAPM_MUX("ADC DMIC MUX", ADC_DIG_CTRL, ADC_DIG_EN, 0, &adc_dmic_src_mux),

	//TX MIXER Left
	SND_SOC_DAPM_MIXER("TX MIXL", SND_SOC_NOPM, 0, 0, tx_left_src_mixer, ARRAY_SIZE(tx_left_src_mixer)),

	//TX MIXER Right
	SND_SOC_DAPM_MIXER("TX MIXR", SND_SOC_NOPM, 0, 0, tx_right_src_mixer, ARRAY_SIZE(tx_right_src_mixer)),

	//RX MIXER
	SND_SOC_DAPM_MIXER("RX MIXER", SND_SOC_NOPM, 0, 0, rx_src_mixer, ARRAY_SIZE(rx_src_mixer)),

	//DAC MIXER
	SND_SOC_DAPM_MIXER("DAC MIXER", DAC_DIG_CTRL, DAC_DIG_EN, 0, dac_src_mixer, ARRAY_SIZE(dac_src_mixer)),

	//LINEOUT PGA
	SND_SOC_DAPM_PGA("LINEOUT PGA", SYS_FUNC_CTRL, DAC_ANA_OUT_EN, 0, NULL, 0),

	//AIF OUT -> (stream widget, stname must be same with codec dai_driver stream_name, which will be used to build dai widget)
	SND_SOC_DAPM_AIF_OUT("AIF ADC OUT", "Capture", 0, SND_SOC_NOPM, 0, 0),

	//AIF IN -> (stream widget, stname must be same with codec dai_driver stream_name, which will be used to build dai widget)
	SND_SOC_DAPM_AIF_IN("AIF DAC IN", "Playback", 0, SND_SOC_NOPM, 0, 0),
};


/*************************************** DAPM routes *******************************************/
//ac102 dapm routes
static const struct snd_soc_dapm_route ac102_dapm_routes[] = {
	//MIC PGA
	{"MIC PGA", NULL, "MICP"},
	{"MIC PGA", NULL, "MICN"},

	//ADC DMIC MUX
	{"ADC DMIC MUX", "ADC switch", "MIC PGA"},
	{"ADC DMIC MUX", "DMIC switch", "DMIC"},

	//TX MIXL
	{"TX MIXL", "RECD_DAT switch", "ADC DMIC MUX"},
	{"TX MIXL", "PLAY_DAT switch", "DAC MIXER"},

	//TX MIXR
	{"TX MIXR", "RECD_DAT switch", "ADC DMIC MUX"},
	{"TX MIXR", "RXM switch", "RX MIXER"},

	//RX MIXER
	{"RX MIXER", "RXL switch", "AIF DAC IN"},
	{"RX MIXER", "RXR switch", "AIF DAC IN"},

	//DAC MIXER
	{"DAC MIXER", "RXM switch", "RX MIXER"},
	{"DAC MIXER", "RECD_DAT switch", "ADC DMIC MUX"},

	//LINEOUT PGA
	{"LINEOUT PGA", NULL, "DAC MIXER"},

	//LINEOUT
	{"LINEOUTP", NULL, "LINEOUT PGA"},
	{"LINEOUTN", NULL, "LINEOUT PGA"},

	//AIF ADC OUT
	{"AIF ADC OUT", NULL, "TX MIXL"},
	{"AIF ADC OUT", NULL, "TX MIXR"},
};


static int ac102_read(u8 reg, u8 *rt_value, struct i2c_client *client)
{
	int ret;
	u8 read_cmd[3] = {0};
	u8 cmd_len = 0;

	read_cmd[0] = reg;
	cmd_len = 1;

	if (client == NULL || client->adapter == NULL){
		pr_err("ac102_read client or client->adapter is NULL\n");
		return -1;
	}

	ret = i2c_master_send(client, read_cmd, cmd_len);
	if (ret != cmd_len) {
		pr_err("ac102_read error1->[REG-0x%02x]\n",reg);
		return -1;
	}

	ret = i2c_master_recv(client, rt_value, 1);
	if (ret != 1) {
		pr_err("ac102_read error2->[REG-0x%02x], ret=%d\n",reg,ret);
		return -1;
	}

	return 0;
}

static int ac102_write(u8 reg, u8 value, struct i2c_client *client)
{
	int ret = 0;
	u8 write_cmd[2] = {0};

	write_cmd[0] = reg;
	write_cmd[1] = value;

	if (client == NULL || client->adapter == NULL){
		pr_err("ac102_write client or client->adapter is NULL\n");
		return -1;
	}

	ret = i2c_master_send(client, write_cmd, 2);
	if (ret != 2) {
		pr_err("ac102_write error->[REG-0x%02x,val-0x%02x]\n",reg,value);
		return -1;
	}

	return 0;
}

static int ac102_update_bits(u8 reg, u8 mask, u8 value, struct i2c_client *client)
{
	u8 val_old;
	u8 val_new;

	ac102_read(reg, &val_old, client);
	val_new = (u8)((val_old & ~mask) | (value & mask));
	if(val_new != val_old){
		ac102_write(reg, val_new, client);
	}

	return 0;
}

#if 0
static int ac102_multi_chips_read(u8 reg, unsigned char *rt_value)
{
	u8 i;

	for(i=0; i<AC102_CHIP_NUMS; i++){
		ac102_read(reg, rt_value++, i2c_ctrl[i]);
	}

	return 0;
}
#endif

static int ac102_multi_chips_write(u8 reg, u8 value)
{
	u8 i;

	for(i=0; i<AC102_CHIP_NUMS; i++){
		ac102_write(reg, value, i2c_ctrl[i]);
	}

	return 0;
}

static int ac102_multi_chips_update_bits(u8 reg, u8 mask, u8 value)
{
	u8 i;

	for(i=0; i<AC102_CHIP_NUMS; i++){
		ac102_update_bits(reg, mask, value, i2c_ctrl[i]);
	}

	return 0;
}


static void ac102_hw_init(int stream, struct i2c_client *i2c)
{
	/*** Chip reset ***/
	//ac102_write(CHIP_AUDIO_RST, 0x34, i2c);	/*0x00=0x34: resets all registers and all digital modules to their default state*/

	/*** Analog voltage enable ***/
	ac102_write(PWR_CTRL1, 0x73, i2c);		/*0x01=0x73: ALDOOUT=1.8V, DLDOOUT=1.2V, MICBISA=2.39V*/
	ac102_write(PWR_CTRL2, 0x1F, i2c);		/*0x02=0x1F: ALDO/DLDO/MICBIAS/VREF/IREF Enable*/

	/*** SYSCLK Config ***/
	ac102_update_bits(SYS_CLK_ENA, 0x1<<ADC_DIG_CLK_EN | 0x1<<AGC_HPF_CLK_EN | 0x1<<DAC_DIG_CLK_EN | 0x1<<EQ_CLK_EN | 0x1<<I2S_CLK_EN,\
					  0x0<<ADC_DIG_CLK_EN | 0x1<<AGC_HPF_CLK_EN | 0x1<<DAC_DIG_CLK_EN | !AC102_EQ_EN<<EQ_CLK_EN | 0x1<<I2S_CLK_EN, i2c);
	ac102_write(ADC_CLK_SET, AC102_ADC_CLK_DIV&0x0F, i2c);							/*ADC_CLK Divide Ratio from SYS_CLK*/
	ac102_write(DAC_CLK_SET, AC102_DAC_CLK_DIV&0x0F, i2c);							/*DAC_CLK Divide Ratio from SYS_CLK*/

	/*** I2S Common Config ***/
	ac102_update_bits(I2S_CTRL, 0x1<<SDO_EN, 0x1<<SDO_EN, i2c);						/*SDO Enable*/
	ac102_update_bits(I2S_BCLK_CTRL, 0x1<<EDGE_TRANSFER, 0x0<<EDGE_TRANSFER, i2c);	/*SDO drive data and SDI sample data at the different BCLK edge*/
	ac102_update_bits(I2S_LRCK_CTRL1, 0x3<<LRCK_PERIODH, ((AC102_LRCK_PERIOD-1) >> 8)<<LRCK_PERIODH, i2c);
	ac102_write(I2S_LRCK_CTRL2, (u8)(AC102_LRCK_PERIOD-1), i2c);					/*config LRCK period*/
	/*Turn to hi-z state (TDM) when not transferring slot*/
	ac102_update_bits(I2S_FMT_CTRL1, 0x1<<TX_SLOT_HIZ | 0x1<<TX_STATE, 0x0<<TX_SLOT_HIZ | 0x1<<TX_STATE, i2c);
	ac102_update_bits(I2S_FMT_CTRL2, 0x7<<SLOT_WIDTH_SEL, (AC102_SLOT_WIDTH/4-1)<<SLOT_WIDTH_SEL, i2c);		/*8/12/16/20/24/28/32bit Slot Width*/
	/*0x0D=0x60: TX MSB first, Transfer 0 after each sample in each slot(sample resolution < slot width), LRCK = 1 BCLK width (short frame), Linear PCM Data Mode*/

	/*** Mixer Config ***/
#if !AC102_DAPM_EN
	ac102_write(I2S_TX_MIX_SRC, 0x05, i2c);					/*0x13=0x05: TX Mixer L/R Source select REC_DAT, and Mixer Gain set 0dB*/

  #if AC102_CHIP_NUMS>1
	if(i2c->addr == 0x33){
		ac102_write(I2S_RX_MIX_SRC, 0x01, i2c);				/*0x18=0x01: RX Mixer Source select RXL, and Mixer Gain set 0dB*/
	} else if(i2c->addr == 0x30){
		ac102_write(I2S_RX_MIX_SRC, 0x02, i2c);				/*0x18=0x02: RX Mixer Source select RXR, and Mixer Gain set 0dB*/
	}
  #else
	ac102_write(I2S_RX_MIX_SRC, 0x0F, i2c);					/*0x18=0x0F: RX Mixer Source select RXL&RXR, and Mixer Gain set -6dB*/
  #endif

	ac102_write(DAC_MIX_SRC, 0x01, i2c);					/*0x1D=0x01: DAC Mixer Source select RXM, and Mixer Gain set 0dB*/
#endif

	if(stream == SNDRV_PCM_STREAM_CAPTURE){
		/*** ADC Analog/Digital Config ***/
		/*DMIC config, ADC Delay Funtion Disable, ADC Digital Part Enable*/
		ac102_update_bits(ADC_DIG_CTRL, !AC102_DAPM_EN<<DIG_MIC_EN | 0x1<<DIG_MIC_OSR | 0x1<<ADOUT_DLY_EN | !AC102_DAPM_EN<<ADC_DIG_EN,\
						!!AC102_DMIC_EN<<DIG_MIC_EN | 0x0<<DIG_MIC_OSR | 0x0<<ADOUT_DLY_EN | 0x1<<ADC_DIG_EN, i2c);
		/*ADC PGA Gain config, ADC & PGA global enable*/
		ac102_update_bits(ADC_ANA_CTRL1, (0x1F*!AC102_DAPM_EN)<<PGA_GAIN_CTRL | !AC102_DAPM_EN<<ADC_PGA_GEN, AC102_PGA_GAIN<<PGA_GAIN_CTRL | 0x1<<ADC_PGA_GEN, i2c);
		/*ADC AGC config*/
		ac102_update_bits(AGC_CTRL, 0x1<<AGC_EN | 0x1<<NOISE_DET_EN, !!AC102_AGC_EN<<AGC_EN | !!AC102_AGC_EN<<NOISE_DET_EN, i2c);

		/*ADC Pattern select, HPF enable config*/
	#if !AC102_DAPM_EN
	  #if !AC102_ADC_PATTERN_SEL
		ac102_update_bits(AGC_CTRL, 0x1<<HPF_EN, 0x1<<HPF_EN, i2c);	/*ADC HPF Enable*/
	  #else
		ac102_update_bits(AGC_CTRL, 0x1<<HPF_EN, 0x0<<HPF_EN, i2c);	/*ADC HPF Disable*/
		ac102_update_bits(ADC_DIG_CTRL, 0x03<<ADC_PTN_SEL, AC102_ADC_PATTERN_SEL<<ADC_PTN_SEL, i2c);
	  #endif
	#endif
	} else if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/*** DAC Analog/Digital Config ***/
		/*DAC Pattern Select, DAC Digital part Enable*/
		ac102_update_bits(DAC_DIG_CTRL, (0x3*!AC102_DAPM_EN)<<DAC_PTN_SEL | !AC102_DAPM_EN<<DAC_DIG_EN, AC102_DAC_PATTERN_SEL<<DAC_PTN_SEL | 0x1<<DAC_DIG_EN, i2c);
		/*DAC analog output path Enable*/
		ac102_update_bits(SYS_FUNC_CTRL, !AC102_DAPM_EN<<DAC_ANA_OUT_EN, 1<<DAC_ANA_OUT_EN, i2c);
		ac102_write(SYS_FUNC_CTRL, 0x55, i2c);
		/*Line-out Differential Mode Enable, Line-out Amplifier Gain set*/
		ac102_update_bits(DAC_ANA_CTRL2, 0x1<<LINE_OUT_DIF_EN | (0xF*!AC102_DAPM_EN)<<LINE_OUT_AMP_GAIN, 0x0<<LINE_OUT_DIF_EN | AC102_LINEOUT_GAIN<<LINE_OUT_AMP_GAIN, i2c);
		ac102_write(DAC_ANA_CTRL1, 0x60, i2c);
		/*DAC EQ config*/
		ac102_update_bits(EQ_CTRL, 0x1<<EQ_EN, !!AC102_EQ_EN<<EQ_EN, i2c);
	}
}


static int ac102_set_sysclk(struct snd_soc_dai *dai, int clk_id, unsigned int freq, int dir)
{
	AC102_DEBUG("\n--->%s\n",__FUNCTION__);

	ac102_multi_chips_update_bits(SYS_CLK_ENA, 1<<SYSCLK_EN, 1<<SYSCLK_EN);

	return 0;
}

static int ac102_set_pll(struct snd_soc_dai *dai, int pll_id, int source, unsigned int freq_in, unsigned int freq_out)
{
	return 0;
}

static int ac102_set_clkdiv(struct snd_soc_dai *dai, int div_id, int div)
{
	u32 i, bclk_div;
	u8 bclk_div_reg_val;

	AC102_DEBUG("\n--->%s\n",__FUNCTION__);
	if(!div_id){	//use div_id to judge Master/Slave mode,  0: Slave mode, 1: Master mode
		AC102_DEBUG("AC102 work as Slave mode, don't need to config BCLK_DIV\n\n");
		return 0;
	}

	bclk_div = div/(AC102_LRCK_PERIOD);		//default PCM mode
	//bclk_div = div/(2*AC102_LRCK_PERIOD);	//I2S/LJ/RJ mode

	for(i=0; i<ARRAY_SIZE(ac102_bclk_div); i++){
		if(ac102_bclk_div[i].real_val == bclk_div){
			bclk_div_reg_val = ac102_bclk_div[i].reg_val;
			AC102_DEBUG("AC102 set BCLK_DIV_[%u]\n\n",bclk_div);
			break;
		}
	}

	if(i == ARRAY_SIZE(ac102_bclk_div)){
		pr_err("AC102 don't support BCLK_DIV_[%u]\n\n",bclk_div);
		return -EINVAL;
	}

	//AC102 set BCLK DIV
	ac102_multi_chips_update_bits(I2S_BCLK_CTRL, 0xf<<BCLKDIV, bclk_div_reg_val<<BCLKDIV);
	return 0;
}

static int ac102_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	u8 i, tx_offset, i2s_mode, lrck_polarity, brck_polarity;
	struct ac102_priv *ac102 = dev_get_drvdata(dai->dev);
	AC102_DEBUG("\n--->%s\n",__FUNCTION__);

	//AC102 config Master/Slave mode
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
#ifdef CONFIG_HOBOT_SOC
		case SND_SOC_DAIFMT_CBS_CFS:    //AC102 Master
                        AC102_DEBUG("AC102 set to work as Master\n");
                        ac102_update_bits(I2S_CTRL, 0x3<<LRCK_IOEN, 0x3<<LRCK_IOEN, ac102->i2c);        //BCLK & LRCK output
                        break;
                case SND_SOC_DAIFMT_CBM_CFM:    //AC102 Slave
                        AC102_DEBUG("AC102 set to work as Slave\n");
                        ac102_update_bits(I2S_CTRL, 0x3<<LRCK_IOEN, 0x0<<LRCK_IOEN, ac102->i2c);        //BCLK & LRCK input
                        break;
#else
		case SND_SOC_DAIFMT_CBM_CFM:	//AC102 Master
			AC102_DEBUG("AC102 set to work as Master\n");
			ac102_update_bits(I2S_CTRL, 0x3<<LRCK_IOEN, 0x3<<LRCK_IOEN, ac102->i2c);	//BCLK & LRCK output
			break;
		case SND_SOC_DAIFMT_CBS_CFS:	//AC102 Slave
			AC102_DEBUG("AC102 set to work as Slave\n");
			ac102_update_bits(I2S_CTRL, 0x3<<LRCK_IOEN, 0x0<<LRCK_IOEN, ac102->i2c);	//BCLK & LRCK input
			break;
#endif
		default:
			pr_err("AC102 Master/Slave mode config error:%u\n\n",(fmt & SND_SOC_DAIFMT_MASTER_MASK)>>12);
			return -EINVAL;
	}
	for(i=0; i<AC102_CHIP_NUMS; i++){	//multi_chips: only one chip set as Master, and the others also need to set as Slave
		if(i2c_ctrl[i] == ac102->i2c) continue;
		ac102_update_bits(I2S_CTRL, 0x3<<LRCK_IOEN, 0x0<<LRCK_IOEN, i2c_ctrl[i]);
	}

	//AC102 config I2S/LJ/RJ/PCM format
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
		case SND_SOC_DAIFMT_I2S:
			AC102_DEBUG("AC102 config I2S format\n");
			i2s_mode = LEFT_JUSTIFIED_FORMAT;
			tx_offset = 1;
			break;
		case SND_SOC_DAIFMT_RIGHT_J:
			AC102_DEBUG("AC102 config RIGHT-JUSTIFIED format\n");
			i2s_mode = RIGHT_JUSTIFIED_FORMAT;
			tx_offset = 0;
			break;
		case SND_SOC_DAIFMT_LEFT_J:
			AC102_DEBUG("AC102 config LEFT-JUSTIFIED format\n");
			i2s_mode = LEFT_JUSTIFIED_FORMAT;
			tx_offset = 0;
			break;
		case SND_SOC_DAIFMT_DSP_A:
			AC102_DEBUG("AC102 config PCM-A format\n");
			i2s_mode = PCM_FORMAT;
			tx_offset = 1;
			break;
		case SND_SOC_DAIFMT_DSP_B:
			AC102_DEBUG("AC102 config PCM-B format\n");
			i2s_mode = PCM_FORMAT;
			tx_offset = 0;
			break;
		default:
			pr_err("AC102 I2S format config error:%u\n\n",fmt & SND_SOC_DAIFMT_FORMAT_MASK);
			return -EINVAL;
	}
	ac102_multi_chips_update_bits(I2S_FMT_CTRL1, 0x3 << MODE_SEL | 0x1 << OFFSET, (u8)(i2s_mode << MODE_SEL | tx_offset << OFFSET));

	//AC102 config BCLK&LRCK polarity
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
		case SND_SOC_DAIFMT_NB_NF:
			AC102_DEBUG("AC102 config BCLK&LRCK polarity: BCLK_normal,LRCK_normal\n");
			brck_polarity = BCLK_NORMAL_DRIVE_N_SAMPLE_P;
			lrck_polarity = LRCK_LEFT_LOW_RIGHT_HIGH;
			break;
		case SND_SOC_DAIFMT_NB_IF:
			AC102_DEBUG("AC102 config BCLK&LRCK polarity: BCLK_normal,LRCK_invert\n");
			brck_polarity = BCLK_NORMAL_DRIVE_N_SAMPLE_P;
			lrck_polarity = LRCK_LEFT_HIGH_RIGHT_LOW;
			break;
		case SND_SOC_DAIFMT_IB_NF:
			AC102_DEBUG("AC102 config BCLK&LRCK polarity: BCLK_invert,LRCK_normal\n");
			brck_polarity = BCLK_INVERT_DRIVE_P_SAMPLE_N;
			lrck_polarity = LRCK_LEFT_LOW_RIGHT_HIGH;
			break;
		case SND_SOC_DAIFMT_IB_IF:
			AC102_DEBUG("AC102 config BCLK&LRCK polarity: BCLK_invert,LRCK_invert\n");
			brck_polarity = BCLK_INVERT_DRIVE_P_SAMPLE_N;
			lrck_polarity = LRCK_LEFT_HIGH_RIGHT_LOW;
			break;
		default:
			pr_err("AC102 config BCLK/LRCLK polarity error:%u\n\n",(fmt & SND_SOC_DAIFMT_INV_MASK)>>8);
			return -EINVAL;
	}
	ac102_multi_chips_update_bits(I2S_BCLK_CTRL,  0x1 << BCLK_POLARITY, (u8)(brck_polarity << BCLK_POLARITY));
	ac102_multi_chips_update_bits(I2S_LRCK_CTRL1, 0x1 << LRCK_POLARITY, (u8)(lrck_polarity << LRCK_POLARITY));

	return 0;
}

static int ac102_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	u8 i, channels_en, sample_resolution;
	unsigned int channels;
	unsigned int lrck_period;
	AC102_DEBUG("\n--->%s\n",__FUNCTION__);

	//AC102 hw init
	for(i=0; i<AC102_CHIP_NUMS; i++){
		ac102_hw_init(substream->stream, i2c_ctrl[i]);
		lrck_period = params_channels(params) *
			params_format(params);
		if (params_rate(params) == 8000)
			lrck_period = 32;
		ac102_write(I2S_LRCK_CTRL2, (u8)(lrck_period-1), i2c_ctrl[i]);
	}

	//AC102 set channels
	channels = params_channels(params);
	for(i=0; i<(channels<=AC102_CHIP_NUMS ? channels : AC102_CHIP_NUMS); i++){
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK){
			ac102_update_bits(I2S_SLOT_CTRL, 0x3 << RX_CHSEL, (u8)((channels - 1) << RX_CHSEL), i2c_ctrl[i]);
			ac102_update_bits(I2S_CTRL, 0x1<<RXEN, 0x1<<RXEN, i2c_ctrl[i]);
			if(i<2)	ac102_write(I2S_RX_CHMP_CTRL, 0xe4, i2c_ctrl[i]);
			else	ac102_write(I2S_RX_CHMP_CTRL, 0x4e, i2c_ctrl[i]);
		} else {
			channels_en = (u8)(AC102_CHIP_NUMS > 1? 1 << i : ((1 << channels) - 1) << i);
			ac102_update_bits(I2S_SLOT_CTRL, 0x3 << TX_CHSEL, (u8)((channels - 1) << TX_CHSEL), i2c_ctrl[i]);
			ac102_write(I2S_TX_CTRL, channels_en, i2c_ctrl[i]);
			ac102_update_bits(I2S_CTRL, 0x1<<TXEN, 0x1<<TXEN, i2c_ctrl[i]);
		}
	}
	for(; i<AC102_CHIP_NUMS; i++){
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK){
			ac102_update_bits(I2S_SLOT_CTRL, 0x3<<RX_CHSEL, 0<<RX_CHSEL, i2c_ctrl[i]);
			ac102_update_bits(I2S_CTRL, 0x1<<RXEN, 0x0<<RXEN, i2c_ctrl[i]);
		} else {
			ac102_update_bits(I2S_SLOT_CTRL, 0x3<<TX_CHSEL, 0<<TX_CHSEL, i2c_ctrl[i]);
			ac102_write(I2S_TX_CTRL, 0, i2c_ctrl[i]);
			ac102_update_bits(I2S_CTRL, 0x1<<TXEN, 0x0<<TXEN, i2c_ctrl[i]);
		}
	}

	//AC102 set sample resorution
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		sample_resolution = 8;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		sample_resolution = 16;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		sample_resolution = 20;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		sample_resolution = 24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		sample_resolution = 32;
		break;
	default:
		dev_err(dai->dev, "AC102 don't supported the sample resolution: %u\n", params_format(params));
		return -EINVAL;
	}
	ac102_multi_chips_update_bits(I2S_FMT_CTRL2, 0x7 << SAMPLE_RESOLUTION, (u8)((sample_resolution / 4 - 1) << SAMPLE_RESOLUTION));

	//AC102 Globle enable
	ac102_multi_chips_update_bits(I2S_CTRL, 0x1<<GEN, 0x1<<GEN);

	return 0;
}

static int ac102_hw_free(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	AC102_DEBUG("\n--->%s\n",__FUNCTION__);

	if(substream->stream == SNDRV_PCM_STREAM_CAPTURE){
		AC102_DEBUG("AC102 Capture path disable\n");
		ac102_multi_chips_update_bits(PWR_CTRL2, 0x1<<MBIAS_EN, 0x0<<MBIAS_EN);							//MICBIAS disable
		ac102_multi_chips_update_bits(ADC_DIG_CTRL, !AC102_DAPM_EN<<ADC_DIG_EN, 0x0<<ADC_DIG_EN);		//ADC Digital disable
		ac102_multi_chips_update_bits(ADC_ANA_CTRL1, !AC102_DAPM_EN<<ADC_PGA_GEN, 0x0<<ADC_PGA_GEN);	//ADC Analog(ADC&PGA) disable
		ac102_multi_chips_update_bits(AGC_CTRL, 0x1<<AGC_EN, 0x0<<AGC_EN);								//ADC AGC disable
	} else if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		AC102_DEBUG("AC102 Playback path disable\n");
		ac102_multi_chips_update_bits(DAC_DIG_CTRL, !AC102_DAPM_EN<<DAC_DIG_EN, 0x0<<DAC_DIG_EN);		//DAC Digital disable
		ac102_multi_chips_update_bits(SYS_FUNC_CTRL, !AC102_DAPM_EN<<DAC_ANA_OUT_EN, 0<<DAC_ANA_OUT_EN);//DAC Analog disable
		ac102_multi_chips_update_bits(EQ_CTRL, 0x1<<EQ_EN, 0x0<<EQ_EN);									//DAC EQ disable
	}

#if AC102_DAPM_EN
	//AC102_DEBUG("AC102 resets all registers and all digital modules to their default state\n\n");
	//ac102_multi_chips_write(CHIP_AUDIO_RST, 0x34);
#endif

	return 0;
}


/*** define  ac102  dai_ops  struct ***/
static struct snd_soc_dai_ops ac102_dai_ops = {
	/*DAI clocking configuration*/
	.set_sysclk = ac102_set_sysclk,
	.set_pll = ac102_set_pll,
	.set_clkdiv = ac102_set_clkdiv,

	/*ALSA PCM audio operations*/
	.hw_params = ac102_hw_params,
	.hw_free = ac102_hw_free,

	/*DAI format configuration*/
	.set_fmt = ac102_set_fmt,
};

/*** define  ac102  dai_driver struct ***/
static struct snd_soc_dai_driver ac102_dai0 = {
	.name = "ac102-pcm0",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = (AC102_CHIP_NUMS>1 ? AC102_CHIP_NUMS : 2),
		.rates = AC102_RATES,
		.formats = AC102_FORMATS,
	},
/*
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = (AC102_CHIP_NUMS>1 ? AC102_CHIP_NUMS : 2),
		.rates = AC102_RATES,
		.formats = AC102_FORMATS,
	},
*/
	.ops = &ac102_dai_ops,

};

#if 0
static struct snd_soc_dai_driver ac102_dai1 = {
	.name = "ac102-pcm1",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = (AC102_CHIP_NUMS>1 ? AC102_CHIP_NUMS : 2),
		.rates = AC102_RATES,
		.formats = AC102_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = (AC102_CHIP_NUMS>1 ? AC102_CHIP_NUMS : 2),
		.rates = AC102_RATES,
		.formats = AC102_FORMATS,
	},
	.ops = &ac102_dai_ops,
};

static struct snd_soc_dai_driver ac102_dai2 = {
	.name = "ac102-pcm2",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = (AC102_CHIP_NUMS>1 ? AC102_CHIP_NUMS : 2),
		.rates = AC102_RATES,
		.formats = AC102_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = (AC102_CHIP_NUMS>1 ? AC102_CHIP_NUMS : 2),
		.rates = AC102_RATES,
		.formats = AC102_FORMATS,
	},
	.ops = &ac102_dai_ops,
};

static struct snd_soc_dai_driver ac102_dai3 = {
	.name = "ac102-pcm3",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = (AC102_CHIP_NUMS>1 ? AC102_CHIP_NUMS : 2),
		.rates = AC102_RATES,
		.formats = AC102_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = (AC102_CHIP_NUMS>1 ? AC102_CHIP_NUMS : 2),
		.rates = AC102_RATES,
		.formats = AC102_FORMATS,
	},
	.ops = &ac102_dai_ops,
};
#endif

static struct snd_soc_dai_driver *ac102_dai[] = {
#if AC102_CHIP_NUMS > 0
	&ac102_dai0,
#endif

#if AC102_CHIP_NUMS > 1
	&ac102_dai1,
#endif

#if AC102_CHIP_NUMS > 2
	&ac102_dai2,
#endif

#if AC102_CHIP_NUMS > 3
	&ac102_dai3,
#endif
};


static int ac102_probe(struct snd_soc_codec *codec)
{
	struct ac102_priv *ac102 = dev_get_drvdata(codec->dev);
	int ret=0;

	codec->control_data = devm_regmap_init_i2c(ac102->i2c, &ac102_regmap_config);
	ret = PTR_RET(codec->control_data);
	if(ret){
		dev_err(codec->dev, "AC102 regmap init I2C Failed: %d\n", ret);
		return ret;
	}
	ac102->codec = codec;

	return 0;
}

static int ac102_remove(struct snd_soc_codec *codec)
{
	return 0;
}


static int ac102_suspend(struct snd_soc_codec *codec)
{
#if AC102_MATCH_DTS_EN
	struct ac102_priv *ac102 = dev_get_drvdata(codec->dev);
	if (regulator_en && !IS_ERR(ac102->vol_supply.vcc)) {
		regulator_disable(ac102->vol_supply.vcc);
		regulator_en = 0;
	}
#endif

	return 0;
}

static int ac102_resume(struct snd_soc_codec *codec)
{
#if AC102_MATCH_DTS_EN
	struct ac102_priv *ac102 = dev_get_drvdata(codec->dev);
        int ret;
	if (!regulator_en && !IS_ERR(ac102->vol_supply.vcc)) {
		ret = regulator_enable(ac102->vol_supply.vcc);
		if(ret != 0)
			pr_err("[AC102] %s: some error happen, fail to enable regulator!\n", __func__);
		regulator_en = 1;
	}
#endif

	return 0;
}


static unsigned int ac102_codec_read(struct snd_soc_codec *codec, unsigned int reg)
{
	//AC102_DEBUG("\n--->%s\n",__FUNCTION__);
	u8 val_r;
	struct ac102_priv *ac102 = dev_get_drvdata(codec->dev);

	ac102_read((u8)reg, &val_r, ac102->i2c);
	return val_r;
}

static int ac102_codec_write(struct snd_soc_codec *codec, unsigned int reg, unsigned int value)
{
	//AC102_DEBUG("\n--->%s\n",__FUNCTION__);
	ac102_multi_chips_write((u8)reg, (u8)value);
	return 0;
}


/*** define  ac102  codec_driver struct ***/
static struct snd_soc_codec_driver ac102_soc_codec_driver = {
	.probe = ac102_probe,
	.remove = ac102_remove,
	.suspend = ac102_suspend,
	.resume = ac102_resume,

#if AC102_CODEC_RW_USER_EN
	.read = ac102_codec_read,
	.write = ac102_codec_write,
#endif
	.component_driver = {
#if AC102_DAPM_EN
	.controls = ac102_controls,
	.num_controls = ARRAY_SIZE(ac102_controls),
	.dapm_widgets = ac102_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(ac102_dapm_widgets),
	.dapm_routes = ac102_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(ac102_dapm_routes),
#endif
	}
};


static ssize_t ac102_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int i = 0, num;
	u8 value_r, value_w, reg, val, flag;

	struct ac102_priv *ac102 = dev_get_drvdata(dev);
	val = (u8)simple_strtoul(buf, NULL, 16);
	flag = (val >> 16) & 0xFF;

	if (flag) {
		reg = (val >> 8) & 0xFF;
		value_w = val & 0xFF;
		printk("\nWrite: start REG:0x%02x,val:0x%02x,count:0x%02x\n", reg, value_w, flag);
		while(flag--){
			ac102_write(reg, value_w, ac102->i2c);
			printk("Write 0x%02x to REG:0x%02x\n", value_w, reg);
			reg++;
		}
	} else {
		reg = (val >> 8) & 0xFF;
		num = val & 0xff;
		printk("\nRead: start REG:0x%02x,count:0x%02x\n", reg, num);

		do {
			value_r = 0;
			ac102_read(reg, &value_r, ac102->i2c);
			printk("REG[0x%02x]: 0x%02x;  ", reg, value_r);
			reg++;
			i++;
			if ((i==num) || (i%4==0))	printk("\n");
		} while (i<num);
	}

	return count;
}

static ssize_t ac102_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 val;
	struct ac102_priv *ac102 = dev_get_drvdata(dev);
	int ssize = 128;
	char *s = buf;

	//printk("/*** AC102 driver version: V1.0 ***/\n");
	//printk("echo flag|reg|val > ac102\n");
	//printk("eg->read start addres=0x06,count=0x10: echo 0610 >ac102\n");
	//printk("eg->write start addres=0x50,value=0x00,count=0x4: echo 45000 >ac102\n");

	ac102_read(CHIP_AUDIO_RST, &val, ac102->i2c);
	s += snprintf(s, ssize, " CHIP_AUDIO_RST: 0x%02x\n", val);
	ac102_read(PWR_CTRL1, &val, ac102->i2c);
	s += snprintf(s, ssize, " PWR_CTRL1: 0x%02x\n", val);
	ac102_read(PWR_CTRL2, &val, ac102->i2c);
	s += snprintf(s, ssize, " PWR_CTRL2: 0x%02x\n", val);
	ac102_read(SYS_FUNC_CTRL, &val, ac102->i2c);
	s += snprintf(s, ssize, " SYS_FUNC_CTRL: 0x%02x\n", val);
	ac102_read(DAC_CLK_SET, &val, ac102->i2c);
	s += snprintf(s, ssize, " DAC_CLK_SET: 0x%02x\n", val);
	ac102_read(SYS_CLK_ENA, &val, ac102->i2c);
	s += snprintf(s, ssize, " SYS_CLK_ENA: 0x%02x\n", val);
	ac102_read(I2S_CTRL, &val, ac102->i2c);
	s += snprintf(s, ssize, " I2S_CTRL: 0x%02x\n", val);
	ac102_read(I2S_BCLK_CTRL, &val, ac102->i2c);
	s += snprintf(s, ssize, " I2S_BCLK_CTRL: 0x%02x\n", val);
	ac102_read(I2S_LRCK_CTRL1, &val, ac102->i2c);
	s += snprintf(s, ssize, " I2S_LRCK_CTRL1: 0x%02x\n", val);
	ac102_read(I2S_LRCK_CTRL2, &val, ac102->i2c);
	s += snprintf(s, ssize, " I2S_LRCK_CTRL2: 0x%02x\n", val);
	ac102_read(I2S_FMT_CTRL1, &val, ac102->i2c);
	s += snprintf(s, ssize, " I2S_FMT_CTRL1: 0x%02x\n", val);
	ac102_read(I2S_FMT_CTRL2, &val, ac102->i2c);
	s += snprintf(s, ssize, " I2S_FMT_CTRL2: 0x%02x\n", val);
	ac102_read(I2S_FMT_CTRL3, &val, ac102->i2c);
	s += snprintf(s, ssize, " I2S_FMT_CTRL3: 0x%02x\n", val);
	ac102_read(I2S_SLOT_CTRL, &val, ac102->i2c);
	s += snprintf(s, ssize, " I2S_SLOT_CTRL: 0x%02x\n", val);
	ac102_read(I2S_RX_CHMP_CTRL, &val, ac102->i2c);
	s += snprintf(s, ssize, " I2S_RX_CHMP_CTRL: 0x%02x\n", val);
	ac102_read(I2S_RX_MIX_SRC, &val, ac102->i2c);
	s += snprintf(s, ssize, " I2S_RX_MIX_SRC: 0x%02x\n", val);
	ac102_read(DAC_DIG_CTRL, &val, ac102->i2c);
	s += snprintf(s, ssize, " DAC_DIG_CTRL: 0x%02x\n", val);
	ac102_read(DAC_DVC, &val, ac102->i2c);
	s += snprintf(s, ssize, " DAC_DVC: 0x%02x\n", val);
	ac102_read(DAC_MIX_SRC, &val, ac102->i2c);
	s += snprintf(s, ssize, " DAC_MIX_SRC: 0x%02x\n", val);
	ac102_read(DAC_ANA_CTRL1, &val, ac102->i2c);
	s += snprintf(s, ssize, " DAC_ANA_CTRL1: 0x%02x\n", val);
	ac102_read(DAC_ANA_CTRL2, &val, ac102->i2c);
	s += snprintf(s, ssize, " DAC_ANA_CTRL2: 0x%02x\n", val);
	ac102_read(DAC_ANA_CTRL3, &val, ac102->i2c);
	s += snprintf(s, ssize, " DAC_ANA_CTRL3: 0x%02x\n", val);
	ac102_read(DAC_ANA_CTRL4, &val, ac102->i2c);
	s += snprintf(s, ssize, " DAC_ANA_CTRL4: 0x%02x\n", val);

	ac102_read(I2S_TX_CTRL, &val, ac102->i2c);
	s += snprintf(s, ssize, " I2S_TX_CTRL: 0x%02x\n", val);
	ac102_read(I2S_TX_CHMP_CTRL, &val, ac102->i2c);
	s += snprintf(s, ssize, " I2S_TX_CHMP_CTRL: 0x%02x\n", val);
	ac102_read(I2S_TX_MIX_SRC, &val, ac102->i2c);
	s += snprintf(s, ssize, " I2S_TX_MIX_SRC: 0x%02x\n", val);

	if (s != buf)
		*(s - 1) = '\n';
	return (s - buf);
}

static DEVICE_ATTR(ac102, 0644, ac102_show, ac102_store);

static struct attribute *ac102_debug_attrs[] = {
	&dev_attr_ac102.attr,
	NULL,
};

static struct attribute_group ac102_debug_attr_group = {
	.name   = "ac102_debug",
	.attrs  = ac102_debug_attrs,
};

static int ac102_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *i2c_id)
{
	struct ac102_priv *ac102;
	int ret = 0;

	ac102 = devm_kzalloc(&i2c->dev, sizeof(struct ac102_priv), GFP_KERNEL);
	if (ac102 == NULL) {
		dev_err(&i2c->dev, "Unable to allocate ac102 private data\n");
		return -ENOMEM;
	}

	ac102->i2c = i2c;
	dev_set_drvdata(&i2c->dev, ac102);

#if AC102_MATCH_DTS_EN
	char *regulator_name = NULL;
	struct device_node *np = i2c->dev.of_node;
	if (!regulator_en) {
		ret = of_property_read_string(np, AC102_REGULATOR_NAME, &regulator_name);
		if (ret) {
			pr_err("get ac102 regulator name failed \n");
		} else {
			ac102->vol_supply.vcc = regulator_get(NULL, regulator_name);
			if (IS_ERR(ac102->vol_supply.vcc) || !ac102->vol_supply.vcc) {
				pr_err("get ac102 audio-3v3 failed, return!\n");
				return -EFAULT;
			}
			regulator_set_voltage(ac102->vol_supply.vcc, 3300000, 3300000);//1800000 uV
			ret = regulator_enable(ac102->vol_supply.vcc);
			if(ret != 0)
				pr_err("[AC102] %s: some error happen, fail to enable regulator!\n", __func__);
			regulator_en = 1;
		}
	}
#endif

	if (i2c_id->driver_data < AC102_CHIP_NUMS) {
		i2c_ctrl[i2c_id->driver_data] = i2c;
		ret = snd_soc_register_codec(&i2c->dev, &ac102_soc_codec_driver, ac102_dai[i2c_id->driver_data], 1);
		if (ret < 0) {
			dev_err(&i2c->dev, "Failed to register ac102 codec: %d\n", ret);
		}
		ac102_update_bits(AGC_CTRL, 0x1<<HPF_EN, 0x1<<HPF_EN, i2c);	//default config ADC HPF Enable
	} else {
		pr_err("The wrong i2c_id number :%d\n", (int)(i2c_id->driver_data));
	}

	ret = sysfs_create_group(&i2c->dev.kobj, &ac102_debug_attr_group);
	if (ret) {
		pr_err("failed to create attr group\n");
	}

	pr_info("%s register success\n", __func__);
	return ret;
}

static int ac102_i2c_remove(struct i2c_client *i2c)
{
	sysfs_remove_group(&i2c->dev.kobj, &ac102_debug_attr_group);
	snd_soc_unregister_codec(&i2c->dev);
	return 0;
}

//I2C devices register method_3: i2c_detect
#if 0
static int ac102_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	u8 ac102_chip_id;
	struct i2c_adapter *adapter = client->adapter;

	ac102_read(CHIP_AUDIO_RST, &ac102_chip_id, client);
	AC102_DEBUG("\nAC102_Chip_ID on I2C-%d:0x%02X\n", adapter->nr, ac102_chip_id);

	if (ac102_chip_id == 0x12) {
		if(client->addr == 0x33) {
			strlcpy(info->type, "ac102_0", I2C_NAME_SIZE);
			return 0;
		} else if (client->addr == 0x30) {
			strlcpy(info->type, "ac102_1", I2C_NAME_SIZE);
			return 0;
		}
	}

	return -ENODEV;
}
#endif


//I2C devices address used in register method_3
#if 0
static unsigned short ac102_i2c_addr[] = {
#if AC102_CHIP_NUMS > 0
	0x33,
#endif

#if AC102_CHIP_NUMS > 1
	0x30,
#endif

#if AC102_CHIP_NUMS > 2
	0x33,
#endif

#if AC102_CHIP_NUMS > 3
	0x30,
#endif

	I2C_CLIENT_END,
};
#endif

//I2C devices register method_1: i2c_board_info (i2c_register_board_info)
//I2C devices register method_2: device tree source (in .dts file)
#if 0
static struct i2c_board_info ac102_i2c_board_info[] = {
#if AC102_CHIP_NUMS > 0
	{I2C_BOARD_INFO("ac102_0", 0x33),},
#endif

#if AC102_CHIP_NUMS > 1
	{I2C_BOARD_INFO("ac102_1", 0x30),},
#endif

#if AC102_CHIP_NUMS > 2
	{I2C_BOARD_INFO("ac102_2", 0x33),},
#endif

#if AC102_CHIP_NUMS > 3
	{I2C_BOARD_INFO("ac102_3", 0x30),},
#endif
};
#endif

//I2C driver and devices match method_1: i2c_device_id
static struct i2c_device_id ac102_i2c_id[] = {
#if AC102_CHIP_NUMS > 0
	{ "ac102_0", 0 },
#endif

#if AC102_CHIP_NUMS > 1
	{ "ac102_1", 1 },
#endif

#if AC102_CHIP_NUMS > 2
	{ "ac102_2", 2 },
#endif

#if AC102_CHIP_NUMS > 3
	{ "ac102_3", 3 },
#endif
	{ }
};
MODULE_DEVICE_TABLE(i2c, ac102_i2c_id);

//I2C driver and devices match method_2: of_device_id (devices tree)
static struct of_device_id ac102_dt_ids[] = {
#if AC102_CHIP_NUMS > 0
	{ .compatible = "ac102_0", },
#endif

#if AC102_CHIP_NUMS > 1
	{ .compatible = "ac102_1", },
#endif

#if AC102_CHIP_NUMS > 2
	{ .compatible = "ac102_2", },
#endif

#if AC102_CHIP_NUMS > 3
	{ .compatible = "ac102_3", },
#endif
	{ }
};
MODULE_DEVICE_TABLE(of, ac102_dt_ids);

static struct i2c_driver ac102_i2c_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = "ac102",
		.owner = THIS_MODULE,
#if !AC102_MATCH_DTS_EN
		.of_match_table = ac102_dt_ids,
#endif
	},
	.probe = ac102_i2c_probe,
	.remove = ac102_i2c_remove,
	.id_table = ac102_i2c_id,
#if AC102_MATCH_DTS_EN
	.address_list = ac102_i2c_addr,
	.detect = ac102_i2c_detect,
#endif
};

static int __init ac102_init(void)
{
	int ret ;
	ret = i2c_add_driver(&ac102_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register ac102 i2c driver : %d \n", ret);

	return ret;
}
module_init(ac102_init);

static void __exit ac102_exit(void)
{
	i2c_del_driver(&ac102_i2c_driver);
}
module_exit(ac102_exit);

MODULE_DESCRIPTION("ASoC ac102 codec driver");
MODULE_AUTHOR("panjunwen");
MODULE_LICENSE("GPL");

