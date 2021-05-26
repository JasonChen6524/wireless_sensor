#ifndef I2S_H            // Guard against multiple inclusion
#define I2S_H

extern bool initI2S(U8 flag);
extern void SinOn(U8 Inten);
extern void SinOff(void);
extern void Intensity(U8 Inten);

#define M98090A_ADD 0x20
#define M98090B_ADD 0x22

#define SAMPRATE 8000		
#define SAMPRATEF 8000.0

//MAX98090 init registers

#define M98090_INIT_DEFAULT                         \
{                                                   \
    {M98090_DEVICE_SHUTDOWN, 0x00}, /* shut down */ \
    {M98090_SYSTEM_CLOCK, 0x10},    /* PCLK = MCLK */\
    {M98090_MASTER_MODE, 0x00},     /* SLAVE MODE */\
    {M98090_INTERFACE_FORMAT, 0x04},/* I2S Standard Buss, left justified, 16 bit */\
    {M98090_TDM_CONTROL, 0x00},     /* Disable TDM */\
    {M98090_TDM_FORMAT, 0x00},      /* Don't have to write this register if TDM disabled */\
    {M98090_IO_CONFIGURATION, 0x03},/* ENABLE SDIN AND SDOUT */\
    {M98090_LINE_INPUT_CONFIG, 0x00},\
    {M98090_LEFT_ADC_MIXER, 0x20},  /* MIC 1 to Left ADC mixer */\
    {M98090_RIGHT_ADC_MIXER, 0x40}, /* MIC 2 to Right ADC mixer */\
    {M98090_RCV_LOUTL_MIXER, 0x01}, /* DAC Left to line out Left */\
    {M98090_LOUTR_MIXER, 0x82},     /* DAC Right to line out Right, Independent control */\
    {M98090_RCV_LOUTL_CONTROL, 0x03},/* Left Mixer gain -12dB*/\
    {M98090_LOUTR_CONTROL, 0x03},   /* Right Mixer gain -12dB*/\
    {M98090_MIC1_INPUT_LEVEL, 0x20},/* MIC1 Gain */\
    {M98090_MIC2_INPUT_LEVEL, 0x20},/* MIC2 Gain */\
    {M98090_INPUT_ENABLE, 0x03},    /* Enable ADCs */\
    {M98090_OUTPUT_ENABLE, 0x0F},   /* Enable DACs and LINE OUTs */\
    {M98090_INTERRUPT_S, 0x00},     /* Disable INTs */\
    {M98090_DEVICE_SHUTDOWN, 0x80}  /* remove shut down */\
}
//  {M98090_DAI_PLAYBACK_LEVEL, 0x01},/* DAC fine Adjustment*/
//    {M98090_RCV_LOUTL_VOLUME, 0x16},/* PGA LINEL gain*/
//    {M98090_LOUTR_VOLUME, 0x16},    /* PGA LINER gain*/


extern U8 sineactive;


#endif
/*************************** End of file ****************************/
