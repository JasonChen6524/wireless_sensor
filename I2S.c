
/*
 * I2S
 *
 *  Created on: 05/01/20
 *      Author: rgraczyk
 */

// $[Library includes]

#include "stdbool.h"
#include "global.h"
#include "em_usart.h"
#include "i2s.h"
#include "i2c.h"
#include "v3.h"
#include "hal-config.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "max98090.h"
#include <math.h>


U16 getSin(struct sin_osc *sinosc);

#define MAXINTEN 34 // Maximum intensity
#define SINMAX	(32768.0)


// Global to signal that a sine wave is on.  The codec SPK volatage (5V) is necessary
// to generate line out voltages.  so the I2C code needs to know about this to control the sensor port +5V
U8 sineactive = false;

//Intensity table has two components.  A 15 bit fine adjustment and a codec gain rough adjustment.
// IntenTable[Inten][0] is fine adjustment, calculated based on course gain and target in a spreadsheet
U16 IntenTable[MAXINTEN+1][2] = {
	{0, 0x80},  //MUTE
	{23307, 0x06},
	{23362, 0x08},
	{24809, 0x09},
	{23418, 0x0A},
	{29272, 0x0A},
	{24868, 0x0B},
	{29012, 0x0B},
	{23473, 0x0C},
	{26407, 0x0C},
	{29342, 0x0C},
	{29411, 0x0E},
	{27836, 0x10},
	{29481, 0x11},
	{29272, 0x12},
	{27902, 0x13},
	{32552, 0x13},
	{29551, 0x14},
	{26407, 0x15},
	{29342, 0x15},
	{32276, 0x15},
	{31381, 0x16},
	{30299, 0x17},
	{32630, 0x17},
	{31158, 0x18},
	{29621, 0x19},
	{31473, 0x19},
	{29700, 0x1A},
	{31350, 0x1A},
	{29411, 0x1B},
	{30882, 0x1B},
	{32352, 0x1B},
	{31931, 0x1C},
	{31455, 0x1D},
	{32766, 0x1D}
};

float SinGain = 0;

// Init both I2S ports, digital sine wave generators, both codecs
// return true if exists
bool initI2S(U8 flag)
{
//USART_InitI2s_TypeDef init0 = USART_INITI2S_DEFAULT;
//USART_InitI2s_TypeDef init1 = USART_INITI2S_DEFAULT;

USART_InitI2s_TypeDef init;

U8 codec_init[][2] = M98090_INIT_DEFAULT;
U8 var[1];
U8 *pvar;
U8 i;

   I2C_READ(M98090A_ADD, M98090_REVISION_ID, var, 1);
   if (var[0]!=0x43) return(false);
   
   I2C_READ(M98090B_ADD, M98090_REVISION_ID, var, 1);
   if (var[0]!=0x43) return(false);
   
 // initialize codecs with default init data
   for (i=0; i<(sizeof(codec_init)/sizeof(codec_init[0]));i++)
   {
      pvar = codec_init[i];
      I2C_WRITE(M98090A_ADD,*pvar,(pvar+1),1); 
      I2C_WRITE(M98090B_ADD,*pvar,(pvar+1),1);     
   }

   if (flag) 
   {
      Intensity(v3combo.inten);
      return(true);
   }
   
   
// set up Processor clock out to codecs
   CMU_ClkOutPinConfig(0,cmuSelect_EXPCLK  , 8, BSP_CLKOUT0_PORT, BSP_CLKOUT0_PIN);  // Set output pin to 10.05MHz
   
   /* Use location 1: TX  - Pin D3, (RX - Pin D2) */
   /*                 CLK - Pin D0, LRCLK - Pin D1   */

   GPIO_PinModeSet(I2S0_BLK_PORT, I2S0_BLK_PIN, gpioModePushPull, 0);
   GPIO_PinModeSet(I2S0_LRCLK_PORT, I2S0_LRCLK_PIN, gpioModePushPull, 0);
   GPIO_PinModeSet(I2S0_MOSI_PORT, I2S0_MOSI_PIN, gpioModePushPull, 0);
   GPIO_PinModeSet(I2S0_MISO_PORT, I2S0_MISO_PIN, gpioModeInput, 1);

   GPIO_PinModeSet(I2S1_BLK_PORT, I2S1_BLK_PIN, gpioModePushPull, 0);
   GPIO_PinModeSet(I2S1_LRCLK_PORT, I2S1_LRCLK_PIN, gpioModePushPull, 0);
   GPIO_PinModeSet(I2S1_MOSI_PORT, I2S1_MOSI_PIN, gpioModePushPull, 0);
   GPIO_PinModeSet(I2S1_MISO_PORT, I2S1_MISO_PIN, gpioModeInput, 1);

   // Route and Enable USART0 as I2S0
   GPIO->USARTROUTE[0].CLKROUTE = (GPIO->USARTROUTE[0].CLKROUTE & ~_GPIO_USART_CLKROUTE_MASK)
		   	   | (I2S0_BLK_PORT << _GPIO_USART_CLKROUTE_PORT_SHIFT
			   | (I2S0_BLK_PIN << _GPIO_USART_CLKROUTE_PIN_SHIFT));

   GPIO->USARTROUTE[0].CSROUTE = (GPIO->USARTROUTE[0].CSROUTE & ~_GPIO_USART_CSROUTE_MASK)
		   	   | (I2S0_LRCLK_PORT << _GPIO_USART_CSROUTE_PORT_SHIFT
			   | (I2S0_LRCLK_PIN << _GPIO_USART_CSROUTE_PIN_SHIFT));

   GPIO->USARTROUTE[0].TXROUTE = (GPIO->USARTROUTE[0].TXROUTE & ~_GPIO_USART_TXROUTE_MASK)
		   	   | (I2S0_MOSI_PORT << _GPIO_USART_TXROUTE_PORT_SHIFT
			   | (I2S0_MOSI_PIN << _GPIO_USART_TXROUTE_PIN_SHIFT));

   GPIO->USARTROUTE[0].RXROUTE = (GPIO->USARTROUTE[0].RXROUTE & ~_GPIO_USART_RXROUTE_MASK)
		   	   | (I2S0_MISO_PORT << _GPIO_USART_RXROUTE_PORT_SHIFT
			   | (I2S0_MISO_PIN << _GPIO_USART_RXROUTE_PIN_SHIFT));

   GPIO->USARTROUTE[0].ROUTEEN = GPIO_USART_ROUTEEN_CLKPEN | GPIO_USART_ROUTEEN_TXPEN
		   	   | GPIO_USART_ROUTEEN_RXPEN | GPIO_USART_ROUTEEN_CSPEN;

   USART_IntClear(USART0, (USART_IF_TXC | USART_IF_TXBL | USART_IF_TXOF | USART_IF_CCF | USART_IF_TXIDLE));
   
   // Route and Enable USART2 as I2S1
   GPIO->USARTROUTE[2].CLKROUTE = (GPIO->USARTROUTE[2].CLKROUTE & ~_GPIO_USART_CLKROUTE_MASK)
		   	   | (I2S1_BLK_PORT << _GPIO_USART_CLKROUTE_PORT_SHIFT
			   | (I2S1_BLK_PIN << _GPIO_USART_CLKROUTE_PIN_SHIFT));

   GPIO->USARTROUTE[2].CSROUTE = (GPIO->USARTROUTE[2].CSROUTE & ~_GPIO_USART_CSROUTE_MASK)
		   	   | (I2S1_LRCLK_PORT << _GPIO_USART_CSROUTE_PORT_SHIFT
			   | (I2S1_LRCLK_PIN << _GPIO_USART_CSROUTE_PIN_SHIFT));

   GPIO->USARTROUTE[2].TXROUTE = (GPIO->USARTROUTE[2].TXROUTE & ~_GPIO_USART_TXROUTE_MASK)
		   	   | (I2S1_MOSI_PORT << _GPIO_USART_TXROUTE_PORT_SHIFT
			   | (I2S1_MOSI_PIN << _GPIO_USART_TXROUTE_PIN_SHIFT));

   GPIO->USARTROUTE[2].RXROUTE = (GPIO->USARTROUTE[2].RXROUTE & ~_GPIO_USART_RXROUTE_MASK)
		   	   | (I2S1_MISO_PORT << _GPIO_USART_RXROUTE_PORT_SHIFT
			   | (I2S1_MISO_PIN << _GPIO_USART_RXROUTE_PIN_SHIFT));

   GPIO->USARTROUTE[2].ROUTEEN = GPIO_USART_ROUTEEN_CLKPEN | GPIO_USART_ROUTEEN_TXPEN
		   	   | GPIO_USART_ROUTEEN_RXPEN | GPIO_USART_ROUTEEN_CSPEN;

   USART_IntClear(USART2, (USART_IF_TXC | USART_IF_TXBL | USART_IF_TXOF | USART_IF_CCF | USART_IF_TXIDLE)); 
 
   /* Configure USARTs for basic I2S operation 8000Hz*/
   init = (USART_InitI2s_TypeDef) USART_INITI2S_DEFAULT;
   init.sync.baudrate = SAMPRATE*32;
   init.sync.enable  = usartEnable;
   
   USART_InitI2s(USART0, &init); 

   /* Configure USARTs for basic I2S operation 8000Hz*/
   init = (USART_InitI2s_TypeDef) USART_INITI2S_DEFAULT;
   init.sync.baudrate = SAMPRATE*32;
   init.sync.enable  = usartEnable;
   USART_InitI2s(USART2, &init);
   
   SinOff();   // mute and disable NVIC USART IRQ

   USART_IntEnable(USART0, USART_IF_TXBL);

   //USART_IntEnable(USART2, USART_IF_TXBL);
 
   USART_IntSet(USART0, USART_IF_TXBL);	// prime the interrupt hardware
   USART_IntSet(USART2, USART_IF_TXBL);	// prime the interrupt hardware
   
   return(true);
   
}

// note - plan to use USART0 IRQ for both USART0 and USART2
/**< USART0_RX IRQ Handler */
void USART0_RX_IRQHandler(void)
{


}

/**< USART0_TX IRQ Handler */
void USART0_TX_IRQHandler(void)
{

   if (USART_StatusGet(USART0) & USART_STATUS_TXBDRIGHT)  
   {
      USART_TxDouble(USART0,getSin(&sinosc[0]));
      USART_TxDouble(USART2,getSin(&sinosc[2]));
      sinosc[0].sample = USART_RxDoubleGet(USART2);
   }
   else 
   {
      USART_TxDouble(USART0,getSin(&sinosc[1]));
      USART_TxDouble(USART2,getSin(&sinosc[3]));
   }
   
   USART_IntClear(USART0, USART_IF_TXBL);

}

#if 0
   /**< USART2_RX IRQ Handler */
void USART2_RX_IRQHandler(void)
{


}

/**< USART2_TX IRQ Handler */
void USART2_TX_IRQHandler(void)
{
// TBD - Change ISR to immediately write latest sine sample into hardware THEN get next one.
// effectively decreasing latency

   if (USART_StatusGet(USART2) & USART_STATUS_TXBDRIGHT)  USART_TxDouble(USART2,getSin(&sinosc[2]));

   else  USART_TxDouble(USART2,getSin(&sinosc[3]));
   
   USART_IntClear(USART2, USART_IF_TXBL);

}
#endif

#define SINAMP (32768.0*0.99)  // digital sine aplitude % full scale
#define SINSFRQ SAMPRATEF    // sampling frequency, float
#define V3PI	3.14159265358979323846  // math.h pi does not resolve, so put it here

//Digital filter sine generator
U16 getSin(struct sin_osc *sinosc)
{
I16 retval;
   
    if (sinosc->hzfreq!=sinosc->oldfreq) // init filter on frequency change
   {
	   sinosc->w0 = 2.0 * V3PI *  ((double) sinosc->hzfreq/SINSFRQ);
	   sinosc->a1 = -2.0 * cos((double) sinosc->w0);
	   sinosc->y2 = SinGain * sin((double) sinosc->w0);  // SinGain is inverted when set
	   sinosc->y1 = 0;
      sinosc->oldfreq = sinosc->hzfreq;
   }
   
   sinosc->y0 = -sinosc->a1 * sinosc->y1 - sinosc->y2;
   sinosc->y2 = sinosc->y1;
   sinosc->y1 = sinosc->y0;

   retval = (I16)sinosc->y0;
   
   return(retval);
   
   
}

//Enable Sine intterrups
void SinOn(U8 Inten)
{
   //ExpSetPins(EN5V_HIGH);
   MAXBoostOn();

   Intensity(Inten);
   sineactive = true;
   NVIC_SetPriority(USART0_TX_IRQn, 2);  // BLE default lib given priority is 4. Raise to 2. (1 causes BLE disconnects)
                                         // This priority prevents missed samples to codec due to preemption
                                         // Alternative is DMA using the Peripheral Reflex
                                         // This could be moved to spp_server_main.c at initio false startup 
                                         // But calls after initI2S() lin the BLE library set the priorities
   NVIC_EnableIRQ( USART0_TX_IRQn );
}

//Disable Sine intterrups
void SinOff(void)
{
   MAXBoostOff();
   sineactive = false;
   //ExpSetPins(EN5V_LOW);
   Intensity(0);  //MUTE  
   NVIC_DisableIRQ( USART0_TX_IRQn );
}

void Intensity(U8 Inten)
{

   if(Inten > MAXINTEN -1) Inten = MAXINTEN;  // In case of errant intensity, Saturate at MAX
   
   SinGain = (float)IntenTable[Inten][0];
   //Trigger coefficient change on all channels
   sinosc[0].oldfreq = 0xFFFF;     
   sinosc[1].oldfreq = 0xFFFF;
   sinosc[2].oldfreq = 0xFFFF; 
   sinosc[3].oldfreq = 0xFFFF;
   
   
   I2C_WRITE(M98090A_ADD,M98090_RCV_LOUTL_VOLUME,(U8*)&IntenTable[Inten][1],1); // LEFT PGA
   I2C_WRITE(M98090B_ADD,M98090_RCV_LOUTL_VOLUME,(U8*)&IntenTable[Inten][1],1); // LEFT PGA    
   
   I2C_WRITE(M98090A_ADD,M98090_LOUTR_VOLUME,(U8*)&IntenTable[Inten][1],1); // RIGHT PGA
   I2C_WRITE(M98090B_ADD,M98090_LOUTR_VOLUME,(U8*)&IntenTable[Inten][1],1); // RIGHT PGA   
   
   
}

