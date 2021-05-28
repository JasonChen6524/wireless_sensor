/*
 * V3.c
 *
 *  Created on: 4/19/2020
 *      Author: rgraczyk
 */
 
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "em_cmu.h"
#include "global.h"
#include "i2c.h"
#include "i2s.h"
#include "v3.h"
#include "spi.h"
#include "app.h"
#include "mscflash.h"
#include "MAX14676E.h"
//#include "em_rmu.h"
//#include "core_cm33.h"
#include "em_usart.h"
#include "SHComm.h"
#include "native_gecko.h"


void v3_proc_message(union v3_message_UNION * pv3msgU);
bool v3_get_sum( union v3_message_UNION * pv3msgU);
void v3_ack_handle(union v3_message_UNION * pv3msgU);
void v3_ack_nohandle(union v3_message_UNION * pv3msgU);
void v3_nack_msg(union v3_message_UNION * pv3msgU);
void v3_info_init(void);

uint32_t Seconds = 0;
U16 v3Handle;     // rolling pointer to flash storage 128 byte segments


struct sin_osc sinosc[4];  // digital sine generator structures (4)

union v3_message_UNION v3msgU, v3msgflashU;

struct v3mesbuf v3CommBuf = {0};

/*
struct v3mesbuf
{
    U8 rxhead;
    U8 rxtail; 
    U8 rx[V3BUFSIZ];
    U8 txhead;
    U8 txtail;
    U8 tx[V3BUFSIZ];
};
*/

// zeroing of structures happens automatically in Cinit on power on, but expicit here
struct v3_status v3status = {0};
struct v3_init v3init = {0};
struct v3_combo v3combo = {0,0,0,0,0,0,0,0,0,0,V3_AWAKE_TIME,V3_OFF_TIME};
struct v3_hdr v3erase = {0};
struct v3_info v3info = {0};
struct v3_combomod v3combomod = {0};
struct v3_sleep v3sleep = {0};

void v3_init(void)
{

   v3Handle = V3_NO_HANDLE;  // default to no handle before init
   
// init digital oscilation Generators
   sinosc[0].hzfreq = 1500;  
   sinosc[1].hzfreq = 1501;  
   sinosc[2].hzfreq = 2500;  
   sinosc[3].hzfreq = 2501;
//need to detect change in freq to init filter coeeficients
   sinosc[0].oldfreq = 0xFFFF;     
   sinosc[1].oldfreq = 0xFFFF;
   sinosc[2].oldfreq = 0xFFFF; 
   sinosc[3].oldfreq = 0xFFFF;    

// set volume (intensity) to zero  
   SinOff();      // Disable I2S ints, stop generation of sine waves 

//initialize the status message
   v3status.magic = V3_HDR_MAGIC;
   v3status.cmd = V3_CMD_STATUS;
   v3status.len = sizeof(v3status);
   v3status.state = V3_STATE_IDLE;
   v3status.inithandle = V3_NO_HANDLE;    // No handle 
   
// initialize v3erase message
   v3erase.magic = V3_HDR_MAGIC;
   v3erase.cmd = V3_CMD_ERASE;
   v3erase.len = sizeof(v3erase);
   v3erase.handle = V3_NO_HANDLE;    // No handle

// initalize Processor user flash
   progdata.fw_major = V3_FW_MAJOR;  //update in case of already written SN
   progdata.fw_minor = V3_FW_MINOR;
   flash_write(); // update flash on processor IC   

// initialize the v3info message
   v3_info_init();
   v3info.reports = spi_get_free();  // pointer to next open flash position
   
   //v3status.spp = STATE_POWERON; // no need to init here, init in spp_server_main.c

// init combomod variables
   v3combomod.pause = MOD_RESUME;
   
}

void v3_info_init(void)
{

 
   v3info.magic = V3_HDR_MAGIC;
   v3info.cmd = V3_CMD_INFO;
   v3info.len = sizeof(v3info);
   v3info.handle  = V3_NO_HANDLE;
   v3info.hw_major = progdata.hw_major;
   v3info.hw_minor = progdata.hw_minor;
   v3info.fw_major = progdata.fw_major;
   v3info.fw_minor = progdata.fw_minor;
   v3info.expver = ExpGetVer(); // IO expander FW version
   
   memcpy(v3info.serial, progdata.serial, sizeof(progdata.serial));
   memcpy(v3info.unique, progdata.unique, sizeof(progdata.unique));  

}


// Fill transmit message Q with buffer data
// returns 0 success
// returns 1 overflow
// returns 0xFF if not conected and in SPP mode
// on overflow, drop data (change later to return if not enough room)
U8 v3_XmitQ(U8 *buf, U8 n)
{
#if 0
	U8 i;
	if (v3status.spp != STATE_SPP_MODE) return(0xFF);
	for (i=0;i<n;i++)
	{
		v3CommBuf.tx[v3CommBuf.txhead++] = buf[i];
      //v3CommBuf.txhead &= V3BUFMASK  // not needed for 256 size circular buffer
		if (v3CommBuf.txhead==v3CommBuf.txtail) return(1);  // buffer overflow
	}
#else
	U8 i;
	char charbuf[512];
	if (v3status.spp != STATE_SPP_MODE) return(0xFF);
	for(i = 0; i < n; i++)
	{
		charbuf[i] = buf[i];
	}
	BLE_AddtoQueue(charbuf, 512, n, __LINE__);
#endif
	return(0);
}


void recv_spp_msg(void)  //receive message parser
{
U8  qSize, buftail, i;
   
   qSize = v3CommBuf.rxhead-v3CommBuf.rxtail;
   if (qSize<V3_HDR_SIZE) return;  // no data or no full msg header to process.
   
   buftail = v3CommBuf.rxtail;
   
   for (i=0; i< V3_HDR_SIZE;i++)
   {
	   v3msgU.v3_buf8[i] = v3CommBuf.rx[buftail++];  //get header without effecting Q
   }
   // no need to mask tail because size is 256 and 8 bit variable.
   
   // Check for start token, index buffer by one if token not seen
   if  (v3msgU.v3msg.magic != V3_HDR_MAGIC) 
   {
       v3CommBuf.rxtail++; // look for magic next in the buffer
       //  Add code here to flag error
       return;
   }
   
   buftail = v3msgU.v3msg.len;
   if (qSize<v3msgU.v3msg.len) return; // full message is not in buffer
   
   //read entire message
      for (i=0; i< buftail;i++) v3msgU.v3_buf8[i] = v3CommBuf.rx[v3CommBuf.rxtail++];
   // no need to mask tail because size is 256 and 8 bit variable.

// now message is in UNION, process message  
#if 1
   if ( v3_get_sum( &v3msgU ))
	   v3_proc_message(&v3msgU);  //checksum is correct
   else
	   v3_nack_msg( &v3msgU );    //checksum is not correct
#else 
   v3_proc_message(&v3msgU);
#endif

}


void v3_log(char *buf)
{
U8 size;
   
   size  =  strlen(buf);
   v3msgU.v3log.magic = V3_HDR_MAGIC;
   v3msgU.v3log.cmd = V3_CMD_LOG;
   v3msgU.v3log.len = size + (V3_HDR_SIZE+1);
   v3msgU.v3log.handle = V3_NO_HANDLE;  //empty handle
   strcpy(v3msgU.v3log.ascii,buf);
   v3_get_sum(&v3msgU);  // put cecksum in place
   v3_XmitQ((U8*)&v3msgU, v3msgU.v3log.len);
}

// Command parser
void v3_proc_message(union v3_message_UNION * pv3msgU)
{
struct rgbcolor ledset; 
//static U16 otaflashaddr;

   switch (pv3msgU->v3msg.cmd)   // some messages give ack, some a response
                                 // consider an array of functions if the switch gets too big
   {
      case V3_CMD_LED:
         ledset.red = pv3msgU->v3led.red;
         ledset.grn = pv3msgU->v3led.grn;
         ledset.blu = pv3msgU->v3led.blu;
         max14676_LED(ledset);
         v3_ack_handle(pv3msgU);  
      break;
      
      case V3_CMD_SPK:
         ExpSpeakerOn(pv3msgU->v3spk.freq,pv3msgU->v3spk.msdur);
         v3_ack_handle(pv3msgU);
      break;

      case V3_CMD_HAP:
         ExpHapOn(pv3msgU->v3hap.msdur);
         v3_ack_handle(pv3msgU);
      break;

      case V3_CMD_INIT:
         memcpy(&v3init, pv3msgU, sizeof (v3init));   // copy message
         v3status.time   = v3init.time;
         v3Handle = v3info.reports;  // pointer to next open flash position
         v3status.inithandle = v3Handle;
         v3_ack_handle(pv3msgU);
      break;

      case V3_CMD_COMBO:
         memcpy(&v3combo, pv3msgU, sizeof (v3combo));   // copy message
         if (v3combo.inten) v3status.state = V3_STATE_COMBO;
            else v3status.state = V3_STATE_IDLE;
         v3_ack_handle(pv3msgU);
      break;


      case V3_CMD_MOD:
         memcpy(&v3combomod, pv3msgU, sizeof (v3combomod));   // copy message
         Intensity(v3combomod.inten);
         v3_ack_handle(pv3msgU);      
      break;
      
      case V3_CMD_ERASE:
      spi_cmd_ce();  // initiate long erase cycle, set spi_erase_active true
      v3info.reports = 0;  // pointer to next open flash position
      v3Handle = V3_NO_HANDLE;
      v3status.inithandle = V3_NO_HANDLE;
      // Ack in V3 state loop when erase is complete
      break;
      
      case V3_CMD_DATA:  // respond with Flash Data at handle
         spi_cmd_read(pv3msgU->v3msg.handle,pv3msgU);
         v3_XmitQ((U8*)pv3msgU, pv3msgU->v3msg.len);
      break;
      
      case V3_CMD_INFO:
         if ( v3Handle!=V3_NO_HANDLE)
         {
        	 v3info.reports =  v3Handle;
        	 printLog("INFO.REP1 %d\r\n", v3info.reports);
         }
         else
         {
        	 printLog("INFO.REP2 %d\r\n", v3info.reports);
         }
         v3_ack_nohandle((union v3_message_UNION*)&v3info);  // return initialize v3infor structure
         //printLog("INFO.REP %d\r\n", v3info.reports);
      break;

      case V3_CMD_SET:  // manufacturing variable setting command

      // initalize Processor user flash
         progdata.fw_major = V3_FW_MAJOR;  //update in case of already written SN
         progdata.fw_minor = V3_FW_MINOR;
         progdata.hw_major = pv3msgU->v3set.hw_major;
         progdata.hw_minor = pv3msgU->v3set.hw_minor;
         memcpy(progdata.serial, pv3msgU->v3set.serial, sizeof(progdata.serial));
         memcpy(progdata.unique, pv3msgU->v3set.unique, sizeof(progdata.unique));
         flash_write(); // update flash on processor IC 
         v3_info_init();  // update v3info structure
         v3_ack_nohandle(pv3msgU);
      break;
      
      
      case V3_CMD_SLEEP:
      memcpy(&v3sleep, pv3msgU, sizeof (v3sleep));   // copy message
      v3_ack_handle(pv3msgU);
      progdata.sleepmode = v3sleep.sleepmode;
      flash_write(); // update flash on processor IC
      break;

      case V3_OTA_DFU:
      // add code to turn off codecs, supplies
      // add code to tell PMIC to set LEDs at a static color
      //       change PMIC LED mode to modulated via I2C I/O expander
      //       tell I/O expander to enter LED DFU modulation mode
      //       Need to code new I/O expander functions
      gecko_cmd_system_reset(2);

      break;

#if 0
      case V3_CMD_OTADAT:
      if (v3status.state != V3_STATE_OTA)
      {
         v3status.state = V3_STATE_OTA;
         v3Handle = V3_NO_HANDLE; 
         v3status.inithandle = V3_NO_HANDLE;    // No handle
         otaflashaddr = 0;
      }
      spi_cmd_write(&otaflashaddr, pv3msgU); // SPI flash store the message at location otaflashaddr
      spi_cmd_read(otaflashaddr,pv3msgU);
      if (v3_get_sum( &v3msgU )) v3_ack_nohandle(pv3msgU);
      else v3_nack_msg( &v3msgU );  //checksum is not correct 
      otaflashaddr++;
      
      // code stub to jump to reset vector of boot loader
      //if (v3msgU.v3otadat.address) = 0xFFFFFFFF - jump to bootloader entry tbd
      break;
#endif

     default:
     break;

   }

}
   
 void v3_ack_handle(union v3_message_UNION * pv3msgU)  // respond, save to flash
 {
   // Message is preserved from received message
   pv3msgU->v3msg.handle = v3Handle;
   //pv3msgU->v3msg.len = V3_HDR_SIZE;  - Now sending and storing entire message
   v3_get_sum(pv3msgU);  // apply Checksum
   v3_XmitQ((U8*)pv3msgU, pv3msgU->v3msg.len);
   spi_cmd_write(&v3Handle, pv3msgU); // SPI flash store the message at location v3Handle
 }
 
void v3_ack_nohandle(union v3_message_UNION * pv3msgU)  // respond with same message, no flash write
 {
    // Message is preserved from received message
   pv3msgU->v3msg.handle = V3_NO_HANDLE;
   //pv3msgU->v3msg.len = V3_HDR_SIZE;
   v3_get_sum(pv3msgU);  // apply Checksum
   v3_XmitQ((U8*)pv3msgU, pv3msgU->v3msg.len);

 }
 
void v3_nack_msg(union v3_message_UNION * pv3msgU)
 {
    // populate ack specific fields, v3Handle is preserved from received message
    pv3msgU->v3nack.cmd = V3_CMD_NACK;
    pv3msgU->v3nack.len = sizeof( pv3msgU->v3nack);
    v3_get_sum(pv3msgU);  // apply Checksum
    v3_XmitQ((U8*)pv3msgU, pv3msgU->v3nack.len);
 }

// returns true if checksum field matches calculated checksum
// populates checksum in union
bool v3_get_sum( union v3_message_UNION * pv3msgU)
{
U16 sum;
U8 i;

   sum = pv3msgU->v3msg.sum; 
   pv3msgU->v3msg.sum = 0;
   
   for(i=0;i<pv3msgU->v3msg.len;i++) pv3msgU->v3msg.sum+=pv3msgU->v3_buf8[i];
   if (sum== pv3msgU->v3msg.sum) return(true);
   return(false);

}


#define  V3_SM_IDLE     0
#define  V3_SM_COMBO    1
#define  V3_SM_PAUSE    2
#define  V3_SM_CHARGE   3
#define  V3_SM_OFF      4
#define  V3_SM_SLEEP    5
#define  V3_SM_PARK     6

#define LED_SM_START 0
#define LED_SM_CHRG  1
#define LED_SM_CONN  2
#define LED_SM_COMBO 3
#define LED_SM_OFF   4
#define LED_SM_ADV   5
#define LED_SM_PAUSE 6


bool tempReading = 0;
// v3 main state machine
void v3_state(void)
{
static U8 v3_sm =  V3_SM_IDLE, beat, btn_count = 0, up = 1, v3_sm_save, v3_state_save;
static U16 timer,freq1,freq2; 
static U8 led_sm = LED_SM_START, wait = 0;

   v3status.time++;
   Seconds++;
   
    // get pins from I/O expander, merge in BIOSENSOR 
   v3status.conn = ((ExpGetPins() & ~v3PINSBIO) | (v3status.conn & v3PINSBIO));
   
   // Temporary code to force cables connected until new cables are made.
   v3status.conn &= (~(v3PINSCONS0|v3PINSCONS1));
   
   v3status.statusb = max14676_GetStatusB();  // Get PMIC registers for reporting
   v3status.batvcell = max14676_GetBatteryChargeVoltage();
   v3status.statusa = max14676_GetStatusA();
   v3status.crate = max14676_GetBatteryCRATE();

// When in maint timer done state, will not exit until battery drains 0.070V from reg point (4.2V)
// So set charge = 100 until state is exited
   if ((v3status.statusa & 0x07) == 0x06) v3status.batsoc = 100;
   else  v3status.batsoc = max14676_GetBatteryCharge();

// override state machine sequencing for 1) charging, 2) low battery
   if (v3status.state < V3_STATE_CHRG) // See if PMIC CHRGIN active, change state globally on charge OR power button
   {
      // Check power button for power down
      if (!(v3status.conn & v3PINSPBTN) && (v3_sm!=V3_SM_OFF)) btn_count++; else btn_count = 0;
      
      if (btn_count > 2)  // beep then shutdown
      {
         SinOff();      // Disable I2S ints, stop generation of sine waves
         ledseq_set(LEDPOWEROFF,1);  // kick off LED 
         fbseq_set(FBPOWEROFF,1);  // Feedback sequence
         led_sm = LED_SM_OFF;  // signal to LED state machine 
         v3_sm = V3_SM_OFF;
      }
   
      if (v3status.statusb & MAX_STATB_CHGIN)  //detect charge in, notify app , wait timeout before charging if in a treatment
      {
         SinOff(); 
         if(v3status.state <= V3_STATE_EQUILIB)  // in a treatment
         {
            v3_sm_save = v3_sm;
            v3_state_save = v3status.state;
            v3status.state = V3_STATE_CHRGW;
            wait = V3_CHRG_PAUSE;
            v3_sm = V3_SM_PARK;
            fbseq_set(FBSHORTDN,2);  // Feedback sequence
            
         } else  // not in a treatment, begin charging
         {
            v3_sm = V3_SM_CHARGE;
            v3status.state = V3_STATE_CHRG;
         }

      } else if (!spi_erase_active  && (v3status.batsoc <= 2) ) 
         {
            ledseq_set(LEDPOWEROFF,1);  // kick off LED 
            fbseq_set(FBPOWEROFF,1);  // Feedback sequence
            led_sm = LED_SM_OFF;  // signal to LED state machine 
            v3_sm = V3_SM_OFF;  // shutdown
         }   
   } 

#if 0 // remove after testing
// TEST CODE FORCE SINE WAVES
v3status.state = V3_STATE_COMBO;
v3combo.inten = 33;
v3combo.time = 1000;
v3combo.b1 = 2500;
v3combo.b2 = 1500;
v3combo.combonum = 0;
//SinOn(v3combo.inten);
#endif

   switch (v3_sm)
   {

   case  V3_SM_IDLE:
	   if (v3combo.awakesec) v3combo.awakesec--;  // decrement power down reporting timers when idle
         else
         {
            if( v3status.inithandle != V3_NO_HANDLE) v3_init(); // set init handle and other init when v3combo.awakesec terminates
            if (v3combo.offsec) v3combo.offsec--;
         }

         if (v3status.state == V3_STATE_COMBO)
         {
            beat = 0;
            sinosc[0].hzfreq = v3combo.b1;  
            sinosc[2].hzfreq = v3combo.b2; 
            freq1 = v3combo.b1 + (v3combo.combonum*10);
            freq2 = v3combo.b2 + (v3combo.combonum*10);
            sinosc[1].hzfreq = freq1 + beat; 
            sinosc[3].hzfreq = freq2 + beat; 

            timer = v3combo.time;
            v3_sm = V3_SM_COMBO;
            SinOn(v3combo.inten);  // enable I2S IRQs to feed Codecs, set intensity
         }

         if (v3combomod.pause == MOD_EQUILIB)
         {
            v3_sm = V3_SM_PAUSE;
            v3status.state = V3_STATE_EQUILIB;
         }

         if (v3sleep.sleepsec)
         {
            v3_sm = V3_SM_SLEEP ;  
         }
       

         if (!v3combo.offsec && !spi_erase_active ) 
         {
            ledseq_set(LEDPOWEROFF,1);  // kick off LED 
            fbseq_set(FBPOWEROFF,1);  // Feedback sequence
            led_sm = LED_SM_OFF;  // signal to LED state machine 
            v3_sm = V3_SM_OFF;  // shutdown
         }

      break;
      
      case V3_SM_COMBO:
         if (up) beat++; else beat--;
         // make sine variable change atomic  (look at doing add(s) before disable to reduce disable time)
         NVIC_DisableIRQ( USART0_TX_IRQn );
         sinosc[1].hzfreq = freq1 + beat; 
         sinosc[3].hzfreq = freq2 + beat;
         NVIC_EnableIRQ( USART0_TX_IRQn );         
         
         if (beat >=10 ) up = 0;  //count down
         if (beat == 0) up = 1;  //count up
         // set intensity

         if (!timer-- || (v3status.state != V3_STATE_COMBO)) 
         {
            SinOff();
            v3_sm = V3_SM_IDLE;
            v3status.state = V3_STATE_IDLE;
         } else if(v3combomod.pause == MOD_PAUSE)
            {
               SinOff();
               v3_sm = V3_SM_PAUSE;
               v3status.state = V3_STATE_COMBOPAUSE;
            }
      break;

      case V3_SM_PAUSE:
         if (v3combomod.offsec) v3combomod.offsec--;
         
         if(v3combomod.pause == MOD_RESUME)
         {
            SinOn(v3combomod.inten);
            v3_sm = V3_SM_COMBO;
            v3status.state = V3_STATE_COMBO;
         } 
         
         if(v3combomod.pause == MOD_IDLE)
         {
            v3_sm = V3_SM_IDLE;
            v3status.state = V3_STATE_IDLE;
         } 
         
         if (!v3combomod.offsec && !spi_erase_active) 
         {
            ledseq_set(LEDPOWEROFF,1);  // kick off LED 
            fbseq_set(FBPOWEROFF,1);  // Feedback sequence
            led_sm = LED_SM_OFF;  // signal to LED state machine
            v3_sm = V3_SM_OFF;  // shutdown
         }
      break;


      case V3_SM_CHARGE:
         if ( !(v3status.statusb & MAX_STATB_CHGIN))
         {
            ledseq_set(LEDPOWEROFF,1);  // kick off LED 
            fbseq_set(FBPOWEROFF,1);  // Feedback sequence
            led_sm = LED_SM_OFF;  // signal to LED state machine
            v3_sm = V3_SM_OFF;  // shutdown
         }
         max14676_ChargerOn(); // debounce chrgin, each rising edge of CHRGIN resets registers
      break;

      case V3_SM_SLEEP:
      
      if (!v3sleep.sleepsec) gecko_cmd_system_reset(0);

      //if (!v3sleep.sleepsec) NVIC_SystemReset();
      //SCB->AIRCR = SCB_AIRCR_SYSRESETREQ_Pos; // reset system ?
      break;
      

      case V3_SM_OFF: //  shutdown after led, beep and haptic  (LED must be longer time)
         if(!ledseqset.num) 
         {
            progdata.sleepmode = V3SLEEP_NORMAL;
            flash_write(); // update flash on processor IC
            ShutDown(); // power off , wait for LED sequence to complete  
         }
      break;
      
      case V3_SM_PARK:  // indiciate to app that charger is plugged in until timeout
         wait--;
         if (!wait)   // transition to charging after timeout
         {
            v3_sm = V3_SM_CHARGE;
            v3status.state = V3_STATE_CHRG;            
         }

         if ( !(v3status.statusb & MAX_STATB_CHGIN))  // charger unplugged before timeout, resume
         {
            v3_sm = v3_sm_save;
            if (v3_sm_save == V3_SM_COMBO) SinOn(v3combomod.inten);
            v3status.state = v3_state_save; 
         }
      break;

   }
   
// periodically check if an erase is finished then ACK
   if(spi_erase_active && !spi_cmd_busy()) 
   {
      spi_erase_active = false;
      v3_ack_nohandle((union v3_message_UNION *) &v3erase);  // ack erase command
   }
  if(tempReading == 0)
   v3status.temp0 = Temperature(MAX30208A_ADR); // read temperature sensor A
   // TEMPORARY - populate second sensor field with first until second sensor HW is available
   //v3status.temp1 = v3status.temp0;
  else
   v3status.temp1 = Temperature(MAX30208B_ADR); // read temperature sensor B
  tempReading = !tempReading;
#if 0
   printLog("Temp A %d\r\n",v3status.temp0/20);
   printLog("Temp B %d\r\n",v3status.temp1/20);
   printLog("CHRG %d\r\n", v3status.batsoc);
   printLog("VOLT %d\r\n", v3status.batvcell);
   printLog("STAT %d\r\n", v3status.statusb);
   printLog("RATE %d\r\n", v3status.crate);
   printLog("STATA 0x%X\r\n", (U16)v3status.statusa);
   printLog("STATB 0x%X\r\n", (U16)v3status.statusb);

   //printLog("USART %d\r\n",sinosc[0].sample);
#endif

   //printLog("CONN %d\r\n",v3status.conn);

#if 1
   v3status.bio_state  = appState;
   v3status.bio_status = calibrationTimer_read();
#endif

   if (v3combo.awakesec) v3_ack_handle((union v3_message_UNION*)&v3status);  // Send status and store message, unsolicited 
      else v3_ack_nohandle((union v3_message_UNION*)&v3status);  // Send status message, unsolicited


//LED STATE MACHINE
   switch (led_sm)
   {
      case  LED_SM_START:
      if (ledseqset.num != LEDPOWERON)  //wait for power on LED sequence to complete
      {
         if ((v3status.spp == STATE_CONNECTED) || (v3status.spp == STATE_SPP_MODE))
         {
            led_sm = LED_SM_CONN;
            ledseq_set(LEDBLECON,LEDFOREVER);
            fbseq_set(FBSHORTUP,1);  // Feedback sequence
         }
         
         if (v3status.spp == STATE_ADVERTISING) led_sm = LED_SM_ADV;
         
         if(v3_sm == V3_SM_CHARGE)
         {
            led_sm = LED_SM_CHRG;
            fbseq_set(FBSHORTUP,1);  // Feedback sequence
         }
      }
      
      break;
 
      case LED_SM_ADV:
         // state of charge LED control
         if ( v3status.batsoc < 25) ledseq_set(LEDCHRG00,LEDFOREVER);
         else if ( v3status.batsoc < 50) ledseq_set(LEDCHRG25,LEDFOREVER);
            else ledseq_set(LEDCHRG50,LEDFOREVER);         
               
         if ((v3status.spp == STATE_CONNECTED) || (v3status.spp == STATE_SPP_MODE))
         {
            led_sm = LED_SM_CONN;
            ledseq_set(LEDBLECON,LEDFOREVER);
            fbseq_set(FBSHORTUP,1);  // Feedback sequence
         }

         if (v3_sm == V3_SM_CHARGE) 
         {
            led_sm = LED_SM_CHRG;
            fbseq_set(FBSHORTUP,1);  // Feedback sequence             
         }
      break;
 
      case LED_SM_CHRG:
         // state of charge LED control
         if ( v3status.batsoc < 25) ledseq_set(LEDCHRG00,LEDFOREVER);
         else if ( v3status.batsoc < 50) ledseq_set(LEDCHRG25,LEDFOREVER);
            else if ( v3status.batsoc < 95) ledseq_set(LEDCHRG50,LEDFOREVER); 
               else  ledseq_set(LEDCHRG95,LEDFOREVER);
         
         if (v3_sm != V3_SM_CHARGE) 
         {
            led_sm = LED_SM_CONN;
            ledseq_set(LEDBLECON,LEDFOREVER);  
         }
      break;

      case LED_SM_CONN:
      if (v3status.spp == STATE_ADVERTISING)
         {
            led_sm = LED_SM_ADV;
            fbseq_set(FBSHORTDN,1);  // Feedback sequence            
         }
         
         if (v3_sm == V3_SM_PAUSE)
         {
            led_sm =  LED_SM_PAUSE;
            ledseq_set(LEDYELPAUSE,LEDFOREVER);
         }

         if (v3_sm == V3_SM_COMBO)
         {
            ledseq_set(LEDORGFLASH,LEDFOREVER); 
            led_sm = LED_SM_COMBO;          
         }
         
         if (v3_sm == V3_SM_CHARGE) 
         {
            led_sm = LED_SM_CHRG;
            fbseq_set(FBSHORTUP,1);  // Feedback sequence             
         }
      break;
     
      case LED_SM_COMBO:
         if (v3_sm != V3_SM_COMBO)
         {
            led_sm = LED_SM_CONN;
            ledseq_set(LEDBLECON,LEDFOREVER);            
         }
      break;
      
      case LED_SM_PAUSE:
         if (v3_sm != V3_SM_PAUSE)
         {
            led_sm = LED_SM_CONN;
            ledseq_set(LEDBLECON,LEDFOREVER);            
         }      
      break;
      
      case LED_SM_OFF:
      break;
   }
 
}


