
/*
 * I2C.c
 *
 *  Created on: 05/01/20
 *      Author: rgraczyk
 */

// $[Library includes]
#include "global.h"
#include "em_i2c.h"
#include "i2c.h"
#include "v3.h"
#include "I2S.h"
#include "mscflash.h"
#include "led.h"
#include "MAX14676E.h"
#include "em_gpio.h"
#include "hal-config.h"
#include <string.h>
//#include "periph.h"

struct ledplay ledseqset = LEDPLAY_INIT

struct fbplay fbseqset = FBPLAY_INIT;

U16 SensorUnplug(U8 addr);
extern void bpt_main_reset(void); //

#define S0 0
#define S1 1
#define S2 2
#define S3 3
#define S4 4
#define S5 5
#define S6 6
#define S7 7

#define SEN_INIT 10
#define SEN_WAIT 10  // tics to wait for retry if temp sensor not seen
#define SEN_CHK  10  // tics to check if sensor is still there
#define SEN_RETRY 10 // tics to retry if sensor present

/*
Call to determine if sensor cable is attached.
function will populate v3status.conn v3PINSBIO bitfield
*/

void sensor_reset(void)
{
   ExpSetPins(EN5V_HIGH|RESET_LOW|MFIO_HIGH);
   wait_ms(500);
   ExpSetPins(EN5V_HIGH|RESET_HIGH|MFIO_HIGH);
   wait_ms(500);
}


void I2C_Sensors(void)
{
   static U8 state = S0;
   static U8 count = 0;
   
   switch(state)
   {
      case S0: // entered from reset 
         ExpSetPins(EN5V_HIGH|RESET_LOW|MFIO_HIGH);  //disable 5V      
         state = S1;
         initI2S(1);
      break;

     
      case S1:  // assume one tick time for temp sensor init
            count = 0;
            state = S3;
            if (SensorUnplug(MAX30208A_ADR))  // sensor present?
            {
               state = S2;  // sensor not seen
               v3status.conn |= v3PINSBIO;  // set bio status bit - SET = disconnect
               if (sineactive) state = S6;  // Leave 5V enabled, but bio not present
            }
      break;
      
      case S2:
         ExpSetPins(EN5V_LOW|RESET_LOW|MFIO_HIGH);  //disable 5V
         state = S5;  // wait to retry
      break;

      case S3:  //Delay to allow biosensor to init, assume one tick
         state = S4;
         ExpSetPins(EN5V_HIGH|RESET_HIGH|MFIO_HIGH);  //enable 5V, remove reset
      break;

      case S4:
      // init biosensor
         bpt_main_reset();
         v3status.conn &= ~v3PINSBIO;  // clear bio status bit - Clear = present
         state = S5;
      break;


      case S5:  //  stay here for awhile then check again for presence.
         count++;
         if (count == SEN_CHK)
         {
            count = 0;
            if (SensorUnplug(MAX30208A_ADR))  // sensor present?
            {
               v3status.conn |= v3PINSBIO;  // set bio status bit - SET = disconnect              
               if (!sineactive) ExpSetPins(EN5V_LOW|RESET_LOW|MFIO_HIGH);  //disable 5V
               state = S6;  // wait to retry
            } 
         }
      break;

      case S6:
         count++;
         if (count == SEN_RETRY)
         {
            ExpSetPins(EN5V_HIGH|RESET_LOW|MFIO_HIGH);  //enable 5V, pull reset
            initI2S(1);  // Re set up the codec
            state = S1;  // go to check temp sensor present
         }
      break;

   }
   
}


void ExpSpeakerOn(U16 freq, U8 time)  // freq in HZ, time in mS
{
U8 var[IO_EXP_CMD_SIZE] = {CMD_SPK, (U8)SPK_FREQ(freq),SPK_TIME(time), 0};

   I2C_CMD_IO(IO_EXP_ADR,var);

}


void ExpHapOn(U8 time)  // time in mS
{
U8 var[IO_EXP_CMD_SIZE] = {CMD_HAP, 0, HAP_TIME(time), 0};

   I2C_CMD_IO(IO_EXP_ADR,var);

}

U8 ExpGetVer(void)
{
U8 var[IO_EXP_CMD_SIZE] = {CMD_VER, 0, 0, 0};

   I2C_CMD_IO(IO_EXP_ADR,var);
   return(var[IO_EXP_CMD_SIZE-1]);

}

U8 ExpGetPins(void)
{
U8 var[IO_EXP_CMD_SIZE] = {CMD_GET, 0, 0, 0};

   I2C_CMD_IO(IO_EXP_ADR,var);
   return(var[IO_EXP_CMD_SIZE-1]);   
 
}

U8 ExpSetPins(U8 bits)
{
 U8 var[IO_EXP_CMD_SIZE] = {CMD_SET, 0, bits, 0};

   I2C_CMD_IO(IO_EXP_ADR,var);
   return 0;                                                        //Jason
}

#if 0
U8 ExpGetMFIO(void)                                                 //Jason
{
U8 var[IO_EXP_CMD_SIZE] = {CMD_GET_MFIO, 0, 0, 0};

   I2C_CMD_IO(IO_EXP_ADR,var);
   return(var[IO_EXP_CMD_SIZE-1]);
}

U8 ExpSetPins_2(U8 bits)                                            //Jason
{
 U8 var[IO_EXP_CMD_SIZE] = {CMD_SET, OUTPUT_HIGH, bits, 0};

   I2C_CMD_IO(IO_EXP_ADR,var);
   return 0;
}

U8 ExpResetPins_2(U8 bits)                                         //Jason
{
 U8 var[IO_EXP_CMD_SIZE] = {CMD_SET, OUTPUT_LOW, bits, 0};

   I2C_CMD_IO(IO_EXP_ADR,var);
   return 0;
}

U8 ExpSetPinsOutput(U8 bits)                                       //Jason
{
 U8 var[IO_EXP_CMD_SIZE] = {CMD_SET_DIR, BIT_OUTPUT, bits, 0};

   I2C_CMD_IO(IO_EXP_ADR,var);
   return 0;
}

U8 ExpSetPinsInput(U8 bits)                                        //Jason
{
 U8 var[IO_EXP_CMD_SIZE] = {CMD_SET_DIR, BIT_INPUT, bits, 0};

   I2C_CMD_IO(IO_EXP_ADR,var);
   return 0;
}

U8 ExpSetIRQ_Enable(void)                                         //Jason
{
 U8 var[IO_EXP_CMD_SIZE] = {CMD_IRQ_ENABLE, 0, 0, 0};

   I2C_CMD_IO(IO_EXP_ADR,var);
   return 0;
}

U8 ExpSetIRQ_Disable(void)                                        //Jason
{
 U8 var[IO_EXP_CMD_SIZE] = {CMD_IRQ_DISABLE, 0, 0, 0};

   I2C_CMD_IO(IO_EXP_ADR,var);
   return 0;
}
#endif

void initI2C(void)
{

  // Using default settings
  I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;

  //i2cInit.freq = 10000;

  // Using PA5 (SCL) and PA6 (SDA)
  GPIO_PinModeSet(BSP_I2C0_SCL_PORT, BSP_I2C0_SCL_PIN, gpioModeWiredAndPullUpFilter, 1);
  GPIO_PinModeSet(BSP_I2C0_SDA_PORT, BSP_I2C0_SDA_PIN, gpioModeWiredAndPullUpFilter, 1);
  //GPIO_PinModeSet(gpioPortA, 5, gpioModeWiredAndPullUp, 1);
  //GPIO_PinModeSet(gpioPortA, 6, gpioModeWiredAndPullUp, 1);

  // Route GPIO pins to I2C module
  GPIO->I2CROUTE[0].SDAROUTE = (GPIO->I2CROUTE[0].SDAROUTE & ~_GPIO_I2C_SDAROUTE_MASK)
                        | (gpioPortA << _GPIO_I2C_SDAROUTE_PORT_SHIFT
                        | (6 << _GPIO_I2C_SDAROUTE_PIN_SHIFT));
  GPIO->I2CROUTE[0].SCLROUTE = (GPIO->I2CROUTE[0].SCLROUTE & ~_GPIO_I2C_SCLROUTE_MASK)
                        | (gpioPortA << _GPIO_I2C_SCLROUTE_PORT_SHIFT
                        | (5 << _GPIO_I2C_SCLROUTE_PIN_SHIFT));
  GPIO->I2CROUTE[0].ROUTEEN = GPIO_I2C_ROUTEEN_SDAPEN | GPIO_I2C_ROUTEEN_SCLPEN;

  // Initializing the I2C
  I2C_Init(I2C0, &i2cInit);

  // Setting the status flags and index
 // i2c_startTx = false;

  I2C0->CTRL = I2C_CTRL_AUTOSN;

  ExpSetPins(EN5V_HIGH|RESET_LOW|MFIO_HIGH);
  max14676_init();
  //ExpSetPins(EN5V_HIGH|RESET_HIGH|MFIO_HIGH);

  Temperature_reset();                                                            // Added by Jason Chen, 2021.04.22
}


int I2C_READ(U8 addr, U8 reg, U8 *data, U16 len)
{
I2C_TransferSeq_TypeDef    seq;
I2C_TransferReturn_TypeDef ret; 

    seq.addr = addr;
    seq.flags = I2C_FLAG_WRITE_READ;
    
    seq.buf[0].len = 1;
    seq.buf[1].len = len;
    seq.buf[0].data = &reg;
    seq.buf[1].data = data;
    
    // Do a polled transfer
    ret = I2C_TransferInit(I2C0, &seq);
    while (ret == i2cTransferInProgress)
    {
      ret = I2C_Transfer(I2C0);
    }

     return ((int) ret);

}

int I2C_WRITE(U8 addr, U8 reg , U8 *data, U16 len)
{
I2C_TransferSeq_TypeDef    seq;
I2C_TransferReturn_TypeDef ret; 

    seq.addr = addr;
    seq.flags = I2C_FLAG_WRITE_WRITE;
    
    seq.buf[0].len = 1;
    seq.buf[1].len = len;
    seq.buf[0].data = &reg;
    seq.buf[1].data = data;
    
    // Do a polled transfer
    ret = I2C_TransferInit(I2C0, &seq);
    while (ret == i2cTransferInProgress)
    {
      ret = I2C_Transfer(I2C0);
    }

     return ((int) ret);
}

int m_i2cBus_read(U8 addr, U8 *data, U16 len)   // Added by Jason for Biosensor read
{
	I2C_TransferSeq_TypeDef    seq;
	I2C_TransferReturn_TypeDef ret;

	    seq.addr = addr;
	    seq.flags = I2C_FLAG_READ;

	    seq.buf[1].len = 0;
	    seq.buf[1].data = NULL;
	    seq.buf[0].len = len;
	    seq.buf[0].data = data;

	    // Do a polled transfer
	    ret = I2C_TransferInit(I2C0, &seq);
	    while (ret == i2cTransferInProgress)
	    {
	      ret = I2C_Transfer(I2C0);
	    }

	     return ((int) ret);

}

int m_i2cBus_write(U8 addr, U8 *cmd_bytes, int cmd_bytes_len, bool flag)               // Added by Jason for Biosensor Write
{
	I2C_TransferSeq_TypeDef    seq;
	I2C_TransferReturn_TypeDef ret;

	    seq.addr = addr;
	    seq.flags = I2C_FLAG_WRITE_WRITE;

	    seq.buf[0].len = cmd_bytes_len;
	    seq.buf[0].data = cmd_bytes;
	    seq.buf[1].len = 0;
	    seq.buf[1].data = NULL;

	    // Do a polled transfer
	    ret = I2C_TransferInit(I2C0, &seq);
	    while (ret == i2cTransferInProgress)
	    {
	      ret = I2C_Transfer(I2C0);
	    }

	     return ((int) ret);
}

int I2C_CMD_IO(U8 addr, U8 *data)
{
I2C_TransferSeq_TypeDef    seq;
I2C_TransferReturn_TypeDef ret;

    seq.addr = addr;
    seq.flags = I2C_FLAG_WRITE_READ;

    seq.buf[0].len = 3;
    seq.buf[1].len = 1;
    seq.buf[0].data = data;
    seq.buf[1].data = data+3;

    // Do a polled transfer
    ret = I2C_TransferInit(I2C0, &seq);
    while (ret == i2cTransferInProgress)
    {
      ret = I2C_Transfer(I2C0);
    }
    return ((int) ret);

}

//IS MAX30208 temperature sensor present
// returns MAX30208_ERROR if unplugged (no temp sensor seen)
U16 SensorUnplug(U8 addr)
{
U8 var[2];
//U16 sample;
   
// See if temperature sensor is present
#if 0
   I2C_READ(addr, MAX30208_ID,var,1 );
   if (var[0] != MAX30208_ID_VAL) return(MAX30208_ERROR);
   return (0);
#else
   I2C_READ(MAX30208A_ADR, MAX30208_ID,var,1 );
   I2C_READ(MAX30208B_ADR, MAX30208_ID,&var[1],1 );
   if ((var[0] == MAX30208_ID_VAL)||(var[1] == MAX30208_ID_VAL))
   {
	   return(0);
   }
   return(MAX30208_ERROR);
#endif
}

void Temperature_reset(void)                                                                           //Jason Chen, 2021.04.22
{
	U8 var = 0x01;
	I2C_WRITE(MAX30208A_ADR, MAX30208_RESET, &var, 1);  // Reset to default status
	I2C_WRITE(MAX30208B_ADR, MAX30208_RESET, &var, 1);  // Reset to default status
}

//Read MAX30208 temperature sensor
U16 Temperature(U8 addr)
{
U8 var[2];
U16 sample;
   
// See if temperature sensor is present
   I2C_READ(addr, MAX30208_ID,var,1 );
   if (var[0] != MAX30208_ID_VAL) return(MAX30208_ERROR);
   
   I2C_READ(addr, MAX30208_STATUS,var,1 );
   
   if (!(var[0] & MAX30208_STATUS_RDY)) // no sample waiting
   {
      var[0] = MAX30208_SETUP_GO;
      I2C_WRITE(addr, MAX30208_SETUP, var, 1);  // initiate a conversion for next temperature read
      return(MAX30208_WAIT);        // RETURN not ready
   }
// GET FIFO level
   I2C_READ(addr, MAX30208_COUNT,var,1 );
   if (var[0] != 0x01)  // a single sample is expected
   {
      var[0] = MAX30208_CFG2_FLUSH;             // Flush FIFO
      I2C_WRITE(addr, MAX30208_CFG2, var,0);
      var[0] = MAX30208_SETUP_GO;
      I2C_WRITE(addr, MAX30208_SETUP, var, 1);  // initiate a conversion for next temperature read
      return(MAX30208_WAIT);        // RETURN not ready
   }

// now actually get sample
   I2C_READ(addr,MAX30208_DATA,var,2);
   sample = (U16)var[1]+((U16)var[0]<<8);
   var[0] = MAX30208_SETUP_GO;
   I2C_WRITE(addr, MAX30208_SETUP, var, 1);  // initiate a conversion for next temperature read
   return(sample);
}

void ShutDown(void)
{
   
   max14676_poweroff();  //does not return 
}

void MAXBoostOn(void)
{
   max14676_Boost(1);   
  
}


void MAXBoostOff(void)
{
   max14676_Boost(0);   
  
}

// select LED sequence to play, repeat = 0xFF is forever
void ledseq_set(U8 num, U8 repeat)
{
static U8 oldseqnum = LEDBLACK;

   if(num > MAXLEDSEQ) return;

   switch(num)
   {
      case LEDBLACK:
         ledseqset.size = 1;  // will trigger ledseq() to send turn off leds
         repeat = 0;
         ledseqset.pos = 0;
      break;
      
      case LEDPOWERON:
         //source = ledturnon;
         ledseqset.seq = (struct rgbcolor*)ledturnon;
         ledseqset.size = (sizeof(ledturnon)/sizeof(ledturnon[0]));
      break;

      case LEDYELFLASH:
         //source = ledyelf;
         ledseqset.seq = (struct rgbcolor*)ledyelf;
         ledseqset.size = (sizeof(ledyelf)/sizeof(ledyelf[0]));
      break;

      case LEDYELPAUSE:
         ledseqset.seq = (struct rgbcolor*)ledyelfs;
         ledseqset.size = (sizeof(ledyelfs)/sizeof(ledyelfs[0]));
      break;
      
      case LEDBLECON:
         //source = ledblecon;
         ledseqset.seq = (struct rgbcolor*)ledblecon;
         ledseqset.size = (sizeof(ledblecon)/sizeof(ledblecon[0]));
      break;

      case LEDORGFLASH:
         //source = ledblecon;
         ledseqset.seq = (struct rgbcolor*)ledorgf;
         ledseqset.size = (sizeof(ledorgf)/sizeof(ledorgf[0]));
      break;
      
      case LEDPOWEROFF:
         //source = ledturnoff;
         ledseqset.seq = (struct rgbcolor*)ledturnoff;
         ledseqset.size = (sizeof(ledturnoff)/sizeof(ledturnoff[0]));
      break;
      
      case LEDCHRG00:
         ledseqset.seq = (struct rgbcolor*)led_00;
         ledseqset.size = (sizeof(led_00)/sizeof(led_00[0]));
      break;
      
      case LEDCHRG25:
         ledseqset.seq = (struct rgbcolor*)led_25;
         ledseqset.size = (sizeof(led_25)/sizeof(led_25[0]));
      break;

      case LEDCHRG50:
         ledseqset.seq = (struct rgbcolor*)led_50;
         ledseqset.size = (sizeof(led_50)/sizeof(led_50[0]));
      break;

      case LEDCHRG95:
         ledseqset.seq = (struct rgbcolor*)led_95;
         ledseqset.size = (sizeof(led_95)/sizeof(led_95[0]));
      break;

      
   }

   if (num!=oldseqnum) ledseqset.pos = 0;
   
   ledseqset.num = num;
   oldseqnum = num;
   ledseqset.repeat  = repeat;
   
   //memcpy(ledseqset.seq, source, count);
   
}


//Led sequencer, added dimming to HW REV02
void ledseq()
{
const struct rgbcolor ledoff = LED_OFF;
struct rgbcolor setled;

   if (!ledseqset.size) return;
   if (progdata.sleepmode == V3SLEEP_OFF) return;  // no LED in sleep off mode

   
   if (!ledseqset.repeat)
   {
      ledseqset.size = 0;
      ledseqset.num = 0;
      max14676_LED(ledoff);
      return;
   }

   setled = ledseqset.seq[ledseqset.pos++];
   if (progdata.sleepmode == V3SLEEP_DIM)  //calculate dim values (will change with REV 03 HW - USE PWM via I2C expander/ PMIC
   {
      if (setled.red>=9) setled.red = (setled.red-7)/2 + 8;
      if (setled.grn>=9) setled.grn = (setled.grn-7)/2 + 8;      
      if (setled.blu>=9) setled.blu = (setled.blu-7)/2 + 8;      
   }
   
   max14676_LED(setled);
   
   if (ledseqset.pos>=ledseqset.size)
   {
      if(ledseqset.repeat != LEDFOREVER) ledseqset.repeat--;
      ledseqset.pos = 0;
   }

}

// select Feedback sequence to play, repeat = 0xFF is forever
void fbseq_set(U8 num, U8 repeat)
{
   if(num > MAXFBSEQ) return;

   switch(num)
   {
      case FBNONE:  // cancels any HAPTIC OR SPEAKER feedback sequencing
      fbseqset.size = 0;
      break;      
      
      case FBPOWERON:
      fbseqset.seq = (struct feedback*)fbturnon;
      fbseqset.size = sizeof(fbturnon)/sizeof(fbturnon[0]);
      break; 

      case FBPOWEROFF:
      fbseqset.seq = (struct feedback*)fbturnoff;
      fbseqset.size = sizeof(fbturnoff)/sizeof(fbturnoff[0]);      
      break;

      case FBSHORTUP:
      fbseqset.seq = (struct feedback*)fbshortup;
      fbseqset.size = sizeof(fbshortup)/sizeof(fbshortup[0]);      
      break;

      case FBSHORTDN:
      fbseqset.seq = (struct feedback*)fbshortdn;
      fbseqset.size = sizeof(fbshortdn)/sizeof(fbshortdn[0]);      
      break;
   }      
   
   fbseqset.pos = 0;
   fbseqset.num = num;
   fbseqset.repeat  = repeat;   
}


// Haptic and Speaker Feedback sequencer
void fbseq(void)
{
   if (progdata.sleepmode != V3SLEEP_NORMAL) return ;  // no beeps or haptic if v3sleep.mode is not normal
   if (!fbseqset.size) return;   

   if (!fbseqset.repeat)
   {
      fbseqset.size = 0;
      fbseqset.num = 0;
      return;
   }  

   if(fbseqset.seq[fbseqset.pos].buztime) ExpSpeakerOn(fbseqset.seq[fbseqset.pos].freq, fbseqset.seq[fbseqset.pos].buztime);
   if(fbseqset.seq[fbseqset.pos].haptime) ExpHapOn(fbseqset.seq[fbseqset.pos].haptime);
   
   fbseqset.pos++;
   
   if (fbseqset.pos>=fbseqset.size)
   {
      if(fbseqset.repeat != FBFOREVER) fbseqset.repeat--;
      ledseqset.pos = 0;
   }  
   
 
}
