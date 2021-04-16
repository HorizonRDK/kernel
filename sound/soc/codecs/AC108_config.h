/*
 * AC108_config.h
 *
 *  Created on: 2017年5月25日
 *      Author: mile
 */


#ifndef AC108_CONFIG_H_
#define AC108_CONFIG_H_

unsigned char Codec_cfg_32khz_16bit_IIS[][2]={
    //softreset
//    {0x00,0x12},
    //Analog voltage enable
    {0x06,0x01},    //PWR_CTRL6:            Enable Analog LDO
    {0x07,0x9b},    //PWR_CTRL7:            VREF faststart Enable, Enable VREF @ 3.4V (5V) or 3.1V (3.3V) (needed for Analog LDO and MICBIAS)
    {0x09,0x81},    //PWR_CTRL9:            VREFP faststart Enable, Enable VREFP (needed by all audio input channels)
    {0xB4,0x0b},    //ANA_ADC3_CTRL7:   DSM low power mode Enable, Control bias current for DSM integrator opamps

    //PLL config
    {0x10,0x4f},    //PLL_CTRL1:            PLL Common voltage Enable, PLL Enable
    {0x11,0x00},    //PLL_CTRL2:            PLL pre-divider factor M1=0, M2=0
    {0x12,0x00},    //PLL_CTRL3:            The 2-High Bit of PLL Loop Divider Factor N(N=240)
    {0x13,0xf0},    //PLL_CTRL4:            The 8-Low Bit of PLL PLL Loop Divider Factor N(N=240)
    {0x14,0x29},    //PLL_CTRL5:            PLL post-divider factor K1=9, K2=1
    {0x18,0x01},    //PLL_LOCK_CTRL:        PLL clk lock enable

    //SYSCLK Config
    {0x20,0x99},    //SYSCLK_CTRL:      PLLCLK Enable, PLL Clock Source Select BCLK, System Clock Source Select PLL, SYSCLK Enable
    {0x21,0x93},    //MOD_CLK_EN:       Module clock enable<I2S, ADC digital, MIC offset Calibration, ADC analog>
    {0x22,0x93},    //MOD_RST_CTRL:         Module reset de-asserted<I2S, ADC digital, MIC offset Calibration, ADC analog>

    //I2S Common Config
    {0x30,0x35},    //I2S_CTRL:             BCLK/LRCK input, SDO1&SD2 enable, Transmitter Block Enable, Globe Enable
    {0x32,0x10},    //I2S_LRCK_CTRL1:       LRCK_POLARITY: Left channel when LRCK is high
//    {0x33,0x0f},    //I2S_LRCK_CTRL2:       LRCK_PERIOD 16
    {0x33,0x1f},    //I2S_LRCK_CTRL2:       LRCK_PERIOD 32
    {0x34,0x1d},    //I2S_FMT_CTRL1:        Left mode (offset 0: LJ mode; offset 1: I2S mode), TX1&TX2 offset by 1 BCLKs to LRCK, Turn to hi-z state (TDM) when not transferring slot
//    {0x35,0x33},    //I2S_FMT_CTRL2:        16bit Slot Width, 16bit Sample Resolution
    {0x35,0x75},    //I2S_FMT_CTRL2:        32bit Slot Width, 24bit Sample Resolution
    {0x36,0x60},    //I2S_FMT_CTRL3:        TX MSB first, Transfer 0 after each sample in each slot(sample resolution < slot width), OUT1&2 normal transfer
    {0x38,0x01},    //I2S_TX1_CTRL1:        TX1 2CHs
    {0x39,0x03},    //I2S_TX1_CTRL2:        TX1 Channel1~Channel2 (slot) enable
    {0x3C,0x04},    //I2S_TX1_CHMP_CTRL1:   TX1 CH1~2 Map to CH1~2 adc sample
    {0x40,0x01},    //I2S_TX2_CTRL1:        TX2 2CHs
    {0x41,0x03},    //I2S_TX2_CTRL2:        TX2 Channel1~Channel2 (slot) enable
    {0x44,0x0e},    //I2S_TX2_CHMP_CTRL1:   TX2 CH1~2 Map to CH3~4 adc sample

    //ADC DIG part Config
    {0x60,0x06},    //ADC_SPRC:         ADC Sample Rate 32KHz
//    {0x60,0x08},    //ADC_SPRC:         ADC Sample Rate 48KHz
    {0x61,0x1f},    //ADC_DIG_EN:       Digital part globe enable, ADCs digital part enable
    {0xBB,0x0f},    //ANA_ADC4_CTRL7:       Gating ADCs CLK de-asserted (ADCs CLK Enable)

    //ADC PGA Gain Config
    {0x90,0x19},    //ANA_PGA1_CTRL:        ADC1 PGA Gain 19dB
    {0x91,0x19},    //ANA_PGA2_CTRL:        ADC2 PGA Gain 19dB
    {0x92,0x19},    //ANA_PGA3_CTRL:        ADC3 PGA Gain 19dB
    {0x93,0x19},    //ANA_PGA4_CTRL:        ADC4 PGA Gain 19dB

    //Enable AAF/ADC/PGA    and UnMute Config
    {0xA0,0x07},    //ANA_ADC1_CTRL1:       ADC1 AAF & ADC enable, ADC1 PGA enable, ADC1 MICBIAS enable and UnMute
    {0xA7,0x07},    //ANA_ADC1_CTRL2:       ADC2 AAF & ADC enable, ADC2 PGA enable, ADC2 MICBIAS enable and UnMute
    {0xAE,0x07},    //ANA_ADC1_CTRL3:       ADC3 AAF & ADC enable, ADC3 PGA enable, ADC3 MICBIAS enable and UnMute
    {0xB5,0x07},    //ANA_ADC1_CTRL4:       ADC4 AAF & ADC enable, ADC4 PGA enable, ADC4 MICBIAS enable and UnMute
};
unsigned char Codec_cfg_48khz_32bit_IIS[][2]={
    //softreset
//    {0x00,0x12},
    //Analog voltage enable
    {0x06,0x01},    //PWR_CTRL6:            Enable Analog LDO
    {0x07,0x9b},    //PWR_CTRL7:            VREF faststart Enable, Enable VREF @ 3.4V (5V) or 3.1V (3.3V) (needed for Analog LDO and MICBIAS)
    {0x09,0x81},    //PWR_CTRL9:            VREFP faststart Enable, Enable VREFP (needed by all audio input channels)
    {0xB4,0x0b},    //ANA_ADC3_CTRL7:   DSM low power mode Enable, Control bias current for DSM integrator opamps

    //PLL config
    {0x10,0x4f},    //PLL_CTRL1:            PLL Common voltage Enable, PLL Enable
    {0x11,0x00},    //PLL_CTRL2:            PLL pre-divider factor M1=0, M2=0
//    {0x12,0x01},    //PLL_CTRL3:            The 2-High Bit of PLL Loop Divider Factor N(N=480)
//    {0x13,0xe0},    //PLL_CTRL4:            The 8-Low Bit of PLL PLL Loop Divider Factor N(N=480)
    {0x12,0x00},    //PLL_CTRL3:            The 2-High Bit of PLL Loop Divider Factor N(N=160)
    {0x13,0xa0},    //PLL_CTRL4:            The 8-Low Bit of PLL PLL Loop Divider Factor N(N=160)
    {0x14,0x29},    //PLL_CTRL5:            PLL post-divider factor K1=9, K2=1
    {0x18,0x01},    //PLL_LOCK_CTRL:        PLL clk lock enable

    //SYSCLK Config
    {0x20,0x99},    //SYSCLK_CTRL:      PLLCLK Enable, PLL Clock Source Select BCLK, System Clock Source Select PLL, SYSCLK Enable
    {0x21,0x93},    //MOD_CLK_EN:       Module clock enable<I2S, ADC digital, MIC offset Calibration, ADC analog>
    {0x22,0x93},    //MOD_RST_CTRL:         Module reset de-asserted<I2S, ADC digital, MIC offset Calibration, ADC analog>

    //I2S Common Config
    {0x30,0x35},    //I2S_CTRL:             BCLK/LRCK input, SDO1&SD2 enable, Transmitter Block Enable, Globe Enable
    {0x32,0x10},    //I2S_LRCK_CTRL1:       LRCK_POLARITY: Left channel when LRCK is high
//    {0x33,0x0f},    //I2S_LRCK_CTRL2:       LRCK_PERIOD 16
    {0x33,0x1f},    //I2S_LRCK_CTRL2:       LRCK_PERIOD 32
    {0x34,0x1d},    //I2S_FMT_CTRL1:        Left mode (offset 0: LJ mode; offset 1: I2S mode), TX1&TX2 offset by 1 BCLKs to LRCK, Turn to hi-z state (TDM) when not transferring slot
//    {0x35,0x33},    //I2S_FMT_CTRL2:        16bit Slot Width, 16bit Sample Resolution
    {0x35,0x75},    //I2S_FMT_CTRL2:        32bit Slot Width, 24bit Sample Resolution
    {0x36,0x60},    //I2S_FMT_CTRL3:        TX MSB first, Transfer 0 after each sample in each slot(sample resolution < slot width), OUT1&2 normal transfer
    {0x38,0x01},    //I2S_TX1_CTRL1:        TX1 2CHs
    {0x39,0x03},    //I2S_TX1_CTRL2:        TX1 Channel1~Channel2 (slot) enable
    {0x3C,0x04},    //I2S_TX1_CHMP_CTRL1:   TX1 CH1~2 Map to CH1~2 adc sample
    {0x40,0x01},    //I2S_TX2_CTRL1:        TX2 2CHs
    {0x41,0x03},    //I2S_TX2_CTRL2:        TX2 Channel1~Channel2 (slot) enable
    {0x44,0x0e},    //I2S_TX2_CHMP_CTRL1:   TX2 CH1~2 Map to CH3~4 adc sample

    //ADC DIG part Config
//    {0x60,0x06},    //ADC_SPRC:         ADC Sample Rate 32KHz
    {0x60,0x08},    //ADC_SPRC:         ADC Sample Rate 48KHz
    {0x61,0x1f},    //ADC_DIG_EN:       Digital part globe enable, ADCs digital part enable
    {0xBB,0x0f},    //ANA_ADC4_CTRL7:       Gating ADCs CLK de-asserted (ADCs CLK Enable)

    //ADC PGA Gain Config
//    {0x90,0x26},    //ANA_PGA1_CTRL:        ADC1 PGA Gain 19dB
//    {0x91,0x26},    //ANA_PGA2_CTRL:        ADC2 PGA Gain 19dB
//    {0x92,0x26},    //ANA_PGA3_CTRL:        ADC3 PGA Gain 19dB
//    {0x93,0x26},    //ANA_PGA4_CTRL:        ADC4 PGA Gain 19dB
    {0x90,0x26},    //ANA_PGA1_CTRL:        ADC1 PGA Gain 0dB
    {0x91,0x26},    //ANA_PGA2_CTRL:        ADC2 PGA Gain 0dB
    {0x92,0x26},    //ANA_PGA3_CTRL:        ADC3 PGA Gain 0dB
    {0x93,0x26},    //ANA_PGA4_CTRL:        ADC4 PGA Gain 0dB

    //Enable AAF/ADC/PGA    and UnMute Config
    {0xA0,0x07},    //ANA_ADC1_CTRL1:       ADC1 AAF & ADC enable, ADC1 PGA enable, ADC1 MICBIAS enable and UnMute
    {0xA7,0x07},    //ANA_ADC1_CTRL2:       ADC2 AAF & ADC enable, ADC2 PGA enable, ADC2 MICBIAS enable and UnMute
    {0xAE,0x07},    //ANA_ADC1_CTRL3:       ADC3 AAF & ADC enable, ADC3 PGA enable, ADC3 MICBIAS enable and UnMute
    {0xB5,0x07},    //ANA_ADC1_CTRL4:       ADC4 AAF & ADC enable, ADC4 PGA enable, ADC4 MICBIAS enable and UnMute

    //test mode
//    {0x66,0x00},
//    {0x7F,0x02},
};

unsigned char Codec_cfg_16khz_16bit_IIS[][2]={
        //softreset
    //    {0x00,0x12},
        //Analog voltage enable
        {0x06,0x01},    //PWR_CTRL6:            Enable Analog LDO
        {0x07,0x9b},    //PWR_CTRL7:            VREF faststart Enable, Enable VREF @ 3.4V (5V) or 3.1V (3.3V) (needed for Analog LDO and MICBIAS)
        {0x09,0x81},    //PWR_CTRL9:            VREFP faststart Enable, Enable VREFP (needed by all audio input channels)
        {0xB4,0x0b},    //ANA_ADC3_CTRL7:   DSM low power mode Enable, Control bias current for DSM integrator opamps

        //PLL config
        {0x10,0x4f},    //PLL_CTRL1:            PLL Common voltage Enable, PLL Enable
        {0x11,0x00},    //PLL_CTRL2:            PLL pre-divider factor M1=0, M2=0
    //    {0x12,0x01},    //PLL_CTRL3:            The 2-High Bit of PLL Loop Divider Factor N(N=480)
    //    {0x13,0xe0},    //PLL_CTRL4:            The 8-Low Bit of PLL PLL Loop Divider Factor N(N=480)
        {0x12,0x01},    //PLL_CTRL3:            The 2-High Bit of PLL Loop Divider Factor N(N=480)
        {0x13,0xe0},    //PLL_CTRL4:            The 8-Low Bit of PLL PLL Loop Divider Factor N(N=480)
        {0x14,0x29},    //PLL_CTRL5:            PLL post-divider factor K1=9, K2=1
        {0x18,0x01},    //PLL_LOCK_CTRL:        PLL clk lock enable

        //SYSCLK Config
        {0x20,0x99},    //SYSCLK_CTRL:      PLLCLK Enable, PLL Clock Source Select BCLK, System Clock Source Select PLL, SYSCLK Enable
        {0x21,0x93},    //MOD_CLK_EN:       Module clock enable<I2S, ADC digital, MIC offset Calibration, ADC analog>
        {0x22,0x93},    //MOD_RST_CTRL:         Module reset de-asserted<I2S, ADC digital, MIC offset Calibration, ADC analog>

        //I2S Common Config
        {0x30,0x35},    //I2S_CTRL:             BCLK/LRCK input, SDO1&SD2 enable, Transmitter Block Enable, Globe Enable
        {0x32,0x10},    //I2S_LRCK_CTRL1:       LRCK_POLARITY: Left channel when LRCK is high
    //    {0x33,0x0f},    //I2S_LRCK_CTRL2:       LRCK_PERIOD 16
        {0x33,0x1f},    //I2S_LRCK_CTRL2:       LRCK_PERIOD 32
        {0x34,0x1d},    //I2S_FMT_CTRL1:        Left mode (offset 0: LJ mode; offset 1: I2S mode), TX1&TX2 offset by 1 BCLKs to LRCK, Turn to hi-z state (TDM) when not transferring slot
    //    {0x35,0x33},    //I2S_FMT_CTRL2:        16bit Slot Width, 16bit Sample Resolution
        {0x35,0x73},    //I2S_FMT_CTRL2:        32bit Slot Width, 16bit Sample Resolution
        {0x36,0x60},    //I2S_FMT_CTRL3:        TX MSB first, Transfer 0 after each sample in each slot(sample resolution < slot width), OUT1&2 normal transfer
        {0x38,0x01},    //I2S_TX1_CTRL1:        TX1 2CHs
        {0x39,0x03},    //I2S_TX1_CTRL2:        TX1 Channel1~Channel2 (slot) enable
        {0x3C,0x04},    //I2S_TX1_CHMP_CTRL1:   TX1 CH1~2 Map to CH1~2 adc sample
        {0x40,0x01},    //I2S_TX2_CTRL1:        TX2 2CHs
        {0x41,0x03},    //I2S_TX2_CTRL2:        TX2 Channel1~Channel2 (slot) enable
        {0x44,0x0e},    //I2S_TX2_CHMP_CTRL1:   TX2 CH1~2 Map to CH3~4 adc sample

        //ADC DIG part Config
    //    {0x60,0x06},    //ADC_SPRC:         ADC Sample Rate 32KHz
        {0x60,0x03},    //ADC_SPRC:         ADC Sample Rate 16KHz
        {0x61,0x1f},    //ADC_DIG_EN:       Digital part globe enable, ADCs digital part enable
        {0xBB,0x0f},    //ANA_ADC4_CTRL7:       Gating ADCs CLK de-asserted (ADCs CLK Enable)

        //ADC PGA Gain Config
        {0x90,0x26},    //ANA_PGA1_CTRL:        ADC1 PGA Gain 19dB
        {0x91,0x26},    //ANA_PGA2_CTRL:        ADC2 PGA Gain 19dB
        {0x92,0x26},    //ANA_PGA3_CTRL:        ADC3 PGA Gain 19dB
        {0x93,0x26},    //ANA_PGA4_CTRL:        ADC4 PGA Gain 19dB


        //Enable AAF/ADC/PGA    and UnMute Config
        {0xA0,0x07},    //ANA_ADC1_CTRL1:       ADC1 AAF & ADC enable, ADC1 PGA enable, ADC1 MICBIAS enable and UnMute
        {0xA7,0x07},    //ANA_ADC1_CTRL2:       ADC2 AAF & ADC enable, ADC2 PGA enable, ADC2 MICBIAS enable and UnMute
        {0xAE,0x07},    //ANA_ADC1_CTRL3:       ADC3 AAF & ADC enable, ADC3 PGA enable, ADC3 MICBIAS enable and UnMute
        {0xB5,0x07},    //ANA_ADC1_CTRL4:       ADC4 AAF & ADC enable, ADC4 PGA enable, ADC4 MICBIAS enable and UnMute

        //test mode
    //    {0x66,0x00},
    //    {0x7F,0x02},
    };
    unsigned char Codec1_cfg_16khz_16bit_IIS[][2]={
        //softreset
    //    {0x00,0x12},
        //Analog voltage enable
        {0x06,0x01},    //PWR_CTRL6:            Enable Analog LDO
        {0x07,0x9b},    //PWR_CTRL7:            VREF faststart Enable, Enable VREF @ 3.4V (5V) or 3.1V (3.3V) (needed for Analog LDO and MICBIAS)
        {0x09,0x81},    //PWR_CTRL9:            VREFP faststart Enable, Enable VREFP (needed by all audio input channels)
        {0xB4,0x0b},    //ANA_ADC3_CTRL7:   DSM low power mode Enable, Control bias current for DSM integrator opamps

        //PLL config
        {0x10,0x4f},    //PLL_CTRL1:            PLL Common voltage Enable, PLL Enable
        {0x11,0x00},    //PLL_CTRL2:            PLL pre-divider factor M1=0, M2=0
    //    {0x12,0x01},    //PLL_CTRL3:            The 2-High Bit of PLL Loop Divider Factor N(N=480)
    //    {0x13,0xe0},    //PLL_CTRL4:            The 8-Low Bit of PLL PLL Loop Divider Factor N(N=480)
        {0x12,0x01},    //PLL_CTRL3:            The 2-High Bit of PLL Loop Divider Factor N(N=480)
        {0x13,0xe0},    //PLL_CTRL4:            The 8-Low Bit of PLL PLL Loop Divider Factor N(N=480)
        {0x14,0x29},    //PLL_CTRL5:            PLL post-divider factor K1=9, K2=1
        {0x18,0x01},    //PLL_LOCK_CTRL:        PLL clk lock enable

        //SYSCLK Config
        {0x20,0x99},    //SYSCLK_CTRL:      PLLCLK Enable, PLL Clock Source Select BCLK, System Clock Source Select PLL, SYSCLK Enable
        {0x21,0x93},    //MOD_CLK_EN:       Module clock enable<I2S, ADC digital, MIC offset Calibration, ADC analog>
        {0x22,0x93},    //MOD_RST_CTRL:         Module reset de-asserted<I2S, ADC digital, MIC offset Calibration, ADC analog>

        //I2S Common Config
        {0x30,0x35},    //I2S_CTRL:             BCLK/LRCK input, SDO1&SD2 enable, Transmitter Block Enable, Globe Enable
        {0x32,0x10},    //I2S_LRCK_CTRL1:       LRCK_POLARITY: Left channel when LRCK is high
    //    {0x33,0x0f},    //I2S_LRCK_CTRL2:       LRCK_PERIOD 16
        {0x33,0x1f},    //I2S_LRCK_CTRL2:       LRCK_PERIOD 32
        {0x34,0x1d},    //I2S_FMT_CTRL1:        Left mode (offset 0: LJ mode; offset 1: I2S mode), TX1&TX2 offset by 1 BCLKs to LRCK, Turn to hi-z state (TDM) when not transferring slot
    //    {0x35,0x33},    //I2S_FMT_CTRL2:        16bit Slot Width, 16bit Sample Resolution
        {0x35,0x73},    //I2S_FMT_CTRL2:        32bit Slot Width, 16bit Sample Resolution
        {0x36,0x60},    //I2S_FMT_CTRL3:        TX MSB first, Transfer 0 after each sample in each slot(sample resolution < slot width), OUT1&2 normal transfer
        {0x38,0x01},    //I2S_TX1_CTRL1:        TX1 2CHs
        {0x39,0x03},    //I2S_TX1_CTRL2:        TX1 Channel1~Channel2 (slot) enable
        {0x3C,0x04},    //I2S_TX1_CHMP_CTRL1:   TX1 CH1~2 Map to CH1~2 adc sample
        {0x40,0x01},    //I2S_TX2_CTRL1:        TX2 2CHs
        {0x41,0x03},    //I2S_TX2_CTRL2:        TX2 Channel1~Channel2 (slot) enable
        {0x44,0x0e},    //I2S_TX2_CHMP_CTRL1:   TX2 CH1~2 Map to CH3~4 adc sample

        //ADC DIG part Config
    //    {0x60,0x06},    //ADC_SPRC:         ADC Sample Rate 32KHz
        {0x60,0x03},    //ADC_SPRC:         ADC Sample Rate 16KHz
        {0x61,0x1f},    //ADC_DIG_EN:       Digital part globe enable, ADCs digital part enable
        {0xBB,0x0f},    //ANA_ADC4_CTRL7:       Gating ADCs CLK de-asserted (ADCs CLK Enable)

        //ADC PGA Gain Config
        {0x90,0x26},    //ANA_PGA1_CTRL:        ADC1 PGA Gain 19dB
        {0x91,0x26},    //ANA_PGA2_CTRL:        ADC2 PGA Gain 19dB
        {0x92,0x26},    //ANA_PGA3_CTRL:        ADC3 PGA Gain 0dB
        {0x93,0x26},    //ANA_PGA4_CTRL:        ADC4 PGA Gain 0dB


        //Enable AAF/ADC/PGA    and UnMute Config
        {0xA0,0x07},    //ANA_ADC1_CTRL1:       ADC1 AAF & ADC enable, ADC1 PGA enable, ADC1 MICBIAS enable and UnMute
        {0xA7,0x07},    //ANA_ADC1_CTRL2:       ADC2 AAF & ADC enable, ADC2 PGA enable, ADC2 MICBIAS enable and UnMute
        {0xAE,0x07},    //ANA_ADC1_CTRL3:       ADC3 AAF & ADC enable, ADC3 PGA enable, ADC3 MICBIAS enable and UnMute
        {0xB5,0x07},    //ANA_ADC1_CTRL4:       ADC4 AAF & ADC enable, ADC4 PGA enable, ADC4 MICBIAS enable and UnMute

        //test mode
    //    {0x66,0x00},
    //    {0x7F,0x02},
};

#endif /* AC108_CONFIG_H_ */
