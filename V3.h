 /*
 * V3.h
 *
 *  Created on: 4/19/2020
 *      Author: rgraczyk
 */
#ifndef V3_H_
#define V3_H_

#define V3_FW_MAJOR 00  // Temporary version numbers for test (decimal)
#define V3_FW_MINOR 03

#define V3_HDR_MAGIC 0xAA55  // token to start message
#define V3_HDR_MAGICL 0x55
#define V3_HDR_MAGICH 0xAA
#define V3_NO_HANDLE 0xFFFF  // token sent for message handle when there is no handle.

#define V3_AWAKE_TIME (60*5)  // initial/ default seconds to report and store if no activity
#define V3_OFF_TIME (10)  // initial/ default seconds stay on without storing in flash
#define V3_CHRG_PAUSE 30  // number of seconds which charge connected can interrupt a treatment

extern void v3_init(void);
extern U8 v3_XmitQ(U8 *buf, U8 n);
extern void recv_spp_msg(void);
extern void v3_log(char *buf);
extern void v3_state(void);

extern U32 Seconds;
extern U16 Sequence;
extern U16 v3Handle;

struct sin_osc
{
    U16 hzfreq;
    U16 oldfreq;
    float w0;
    float a1;
    float y0;
    float y1;
    float y2;
	U16 sample;
 };
 
extern struct sin_osc sinosc[4];

// v3 message buffers
#define V3BUFSIZ 256
#define V3BUFMASK (V3BUFSIZ-1)
#define V3_MAX_SIZE 128 // max size of a message including header
#define V3_HDR_SIZE 8   // Message header size
#define V3_LEN_POS 3	// byte memory position of the message LEN field
#define V3_FLASH_CMD_SIZ 4 // SPI flash command plus address size
#define V3_FLASHBUF_SIZ ( V3_MAX_SIZE + V3_FLASH_CMD_SIZ) // SPI flash message size
#define V3_FLASH_END 0xFFFF  // last location in flash (65K of 128 bytes)

struct v3mesbuf
{
   U8 rxhead;
   U8 rxtail;
   U8 rx[V3BUFSIZ];
   U8 txhead;
   U8 txtail;
   U8 tx[V3BUFSIZ];
};

extern struct v3mesbuf v3CommBuf;


// BLE SPP connection state set in spp_server_main.c
#define STATE_POWERON	0
#define STATE_ADVERTISING 1
#define STATE_CONNECTED   2
#define STATE_SPP_MODE    3


typedef enum  // order is important for VM state machine override code
{
   V3_STATE_COMBO,
   V3_STATE_COMBOPAUSE,
   V3_STATE_EQUILIB,
   V3_STATE_SLEEP,
   V3_STATE_LOWPOWER,
   V3_STATE_IDLE,
   V3_STATE_CHRG,
   V3_STATE_CHRGW,
   V3_STATE_OTA
} V3_STATE_TypeDef;

// V3 SPP command list

typedef enum
{
   V3_CMD_NULL,
   V3_CMD_INIT,
   V3_CMD_SPK,
   V3_CMD_HAP,
   V3_CMD_LED,
   V3_CMD_ERASE,
   V3_CMD_RESET,
   V3_CMD_COMBO,
   V3_CMD_DATA,
   V3_CMD_INFO,
   V3_CMD_STATUS,
   V3_CMD_LOG,
   V3_CMD_NACK,    // Bad Checksum or other error
   V3_CMD_SET,	// set variables
   V3_CMD_MOD,	//Modify Combo, pause resume or intensity change
   V3_CMD_SLEEP,
   V3_OTA_DFU
//   V3_CMD_OTADAT
} V3_CMD_TypeDef;

//connections emueration
typedef enum
{
   v3CONNTEMP0 = 1,
   v3CONNTEMP1 = 2,
   v3CONNELECT0 = 4,
   v3CONNELECT1 = 8,
   v3CONNCHARGE = 128
} V3_CONN_TypeDef;


// CMD_GET  (v3status.conn) bit mappings
//  D7 | D6   | D5  |  D4  |  D3  |  D2 |  D1 | D0 |
//P_INT|P_ALRT|P_BTN|CON_S1|CON_S0|RESET| BIO |MFIO|  // BIO sensor bit is inserted by the V3

//IO expander pins enumeration
typedef enum
{
   v3PINSMFIO  = 0x01,
   v3PINSBIO   = 0x02,
   v3PINSRESET = 0x04,
   v3PINSCONS0 = 0x08,
   v3PINSCONS1 = 0x10,
   v3PINSPBTN  = 0x20,
   v3PINSPALRT = 0x40,
   v3PINSPINT  = 0x80   
} V3_PINS_TypeDef;

 
struct v3_hdr  // generic message header only
{
   U16 magic;  // key number
   U8 cmd;     // message command - definition of payload data
   U8 len;     // total message payload is V3_MAX_SIZE-V3_HDR_SIZE
   U16 handle; // rolling 16 bit message handle
   U16 sum;    // lower 16 bits of sum of entire message packet with sum = 0;
};


struct v3_message  // generic variable length message structure
{
   U16 magic;  // key number
   U8 cmd;     // message command - definition of payload data
   U8 len;     // total message payload is V3_MAX_SIZE-V3_HDR_SIZE
   U16 handle; // rolling 16 bit message handle
   U16 sum;    // lower 16 bits of sum of entire message packet with sum = 0;
   U8 data[V3_MAX_SIZE-V3_HDR_SIZE];
};

struct v3_info
{
   U16 magic;
   U8 cmd;
   U8 len;
   U16 handle;
   U16 sum;    // lower 16 bits of sum of entire message packet with sum = 0;
   U8 hw_major;
   U8 hw_minor;
   U8 fw_major;
   U8 fw_minor;
   U8 serial[6]; // serial number
   U8 unique[8]; // private device unique number
   U8 expver;  // FW version of the I/O expander
   U8 post;   // error code seen at post
   U16 reports;  // this is the handle (flash address) to be reported
};

extern struct v3_info v3info;


struct v3_status	// rearrange and pack later adding to the end for now
{
   U16 magic;  // key number
   U8 cmd;     // message command - definition of payload data
   U8 len;     // total message payload is V3_MAX_SIZE-V3_HDR_SIZE
   U16 handle; // rolling 16 bit message handle
   U16 sum;    // lower 16 bits of sum of entire message packet with sum = 0;
   U32 time;  // report device time
   U16 inithandle; // handle of the init
   U8 batsoc;   // 8 bit state of charge value read from PMIC fuel Gauge - 0 to 100
   U8 statusb;	// PMIC StatusB register
   U16 batvcell;// 16 bit battery voltage value read from PMIC fuel Gauge, 1 LSb = 78.125uV
   U16 temp0;	// Tethered sensor 0 (Left) = deg C *200
   U16 temp1;	// Tethered sensor 1 (Right) = deg C *200
   U16 temp2;	// Alternate TEMP sensor TBD
   U16 temp3;
   U8 error;   // run time error codes
   U8 state;  // Treatment state V3 is in
   U8 spp;	// BLE connection state
   U8 conn;   // connection bit field for various cables
   U16 crate;	//  Rate of battery charge or discharge, 1 LSb - 0.208% per hour
   U8 statusa;	// PMIC StatusA register  
   U8 bio_status;
   U8 bio_sys_bp;
   U8 bio_dia_bp;
   U8 bio_prog;
   U16 bio_hr; 
   U16 bio_spo2;
   U8 bio_state;
   U16 bio_red;  // RED LED 16 bit scalar from max30101_data_rx() (finger on detector)
};

extern struct v3_status v3status;

struct v3_init
{
   U16 magic;
   U8 cmd;
   U8 len;
   U16 handle;
   U16 sum;    // lower 16 bits of sum of entire message packet with sum = 0;
   U32 time; // Set offset time from Jan 1 2020 12:00am UTC
   U32 user_id; // Store reference to cloud user id
   U32 account_id; // Store reference to cloud account id
};

extern struct v3_init v3init;

struct v3_spk
{
   U16 magic;
   U8 cmd;
   U8 len;
   U16 handle;
   U16 sum;    // lower 16 bits of sum of entire message packet with sum = 0;   
   U16 freq;   // Frequency in Hz
   U8 msdur;   // duration in milliseconds
};

struct v3_hap
{
   U16 magic;
   U8 cmd;
   U8 len;
   U16 handle;
   U16 sum;    // lower 16 bits of sum of entire message packet with sum = 0;   
   U16 inten;  // Intensity 0-5  (use 5 for 1.8V protos)
   U8 msdur;   // duration in milliseconds
};

struct v3_led
{
   U16 magic;
   U8 cmd;
   U8 len;
   U16 handle;
   U16 sum;    // lower 16 bits of sum of entire message packet with sum = 0;   
   U8 red;     // 8 bit (0-255) red intensity , lower 3 bits ignored
   U8 grn;     // 8 bit (0-255) green intensity , lower 3 bits ignored
   U8 blu;     // 8 bit (0-255) blue intensity , lower 3 bits ignored
};

struct v3_log   // Send ASCII text to app
{
    U16 magic;  // key number
    U8 cmd;     // message command - definition of payload data
    U8 len;     // total message payload is V3_MAX_SIZE-V3_HDR_SIZE
    U16 handle; // rolling 16 bit message handle
    U16 sum;    // lower 16 bits of sum of entire message packet with sum = 0;
    char ascii[V3_MAX_SIZE-V3_HDR_SIZE];// zero terminated ASCII text string to be displyed on a diagnostics page
};

struct v3_combo
{
   U16 magic;
   U8 cmd;
   U8 len;
   U16 handle;
   U16 sum;    // lower 16 bits of sum of entire message packet with sum = 0;
   U16 time; 	// number of seconds to run combo
   U16 b1;		// base frequency 1
   U16 b2;		// base frequency 2
   U8 combonum; // which combo are we running 0-7 (user will see 1-8)
   U8 inten;	// treatment intenisity where 0 is off/HALT 
   U16 awakesec;// time to report after combo completed , seconds
   U16 offsec;	// time to stay on, not reporting, seconds
};

extern struct v3_combo v3combo;

#define MOD_RESUME 0
#define MOD_PAUSE 1
#define MOD_EQUILIB 2
#define MOD_IDLE 3

struct v3_combomod
{
   U16 magic;
   U8 cmd;
   U8 len;
   U16 handle;
   U16 sum;    // lower 16 bits of sum of entire message packet with sum = 0;
   U8 inten;	// treatment intensity where 0 is off
   U8 pause;	// 0 = resume previous combo, 1 = pause combo, 2 = temperature equilibrium, 3 = return to idle from equilibrium
   U16 offsec;  // time to sit in Pause/equilibrium before poweroff in seconds
   
};

struct v3_set	// manufacturing command to set hardware variables
{
   U16 magic;
   U8 cmd;
   U8 len;
   U16 handle;
   U16 sum;    // lower 16 bits of sum of entire message packet with sum = 0;	
   U8 hw_major;
   U8 hw_minor;	
   U8 serial[6];	
   U8 unique[8]; // private device unique number
};

#define V3SLEEP_NORMAL 0  // Haptic, LEDs and beeps normal
#define V3SLEEP_DIM 1	// LEDs (DIM), no haptic, no beeps
#define V3SLEEP_OFF 2	// NO HAPTIC, LED, or beeps

struct v3_sleep	// night time treatment
{
   U16 magic;
   U8 cmd;
   U8 len;
   U16 handle;
   U16 sum;       // lower 16 bits of sum of entire message packet with sum = 0;	
   U16 sleepsec;  // number of seconds to sleep 65535 max
   U16 offsec;    // after wake, number of seconds to stay awake before turning off
   U8 sleepmode;  // User feedback settings (Haptic, LED, Beeper) upon waking
};

extern struct v3_sleep v3sleep;

#if 0 
struct v3_otadat
{
   U16 magic;
   U8 cmd;
   U8 len;
   U16 handle;
   U16 sum;    // lower 16 bits of sum of entire message packet with sum = 0;
   U32 address; // V3 flash memory address
   U8 data[V3_MAX_SIZE-V3_HDR_SIZE-sizeof(U32)];	
};
#endif

union v3_message_UNION
{
    U8 v3_buf8[V3_MAX_SIZE];
   
   struct v3_message v3msg;
   struct v3_info    v3info;
   struct v3_status  v3stat;
   struct v3_init    v3init;
   struct v3_spk     v3spk;	
   struct v3_hap     v3hap;
   struct v3_combo   v3combo;
   struct v3_combomod v3combomod;
   struct v3_log     v3log;
   struct v3_led     v3led;
   struct v3_hdr     v3hdr;
   struct v3_set     v3set;
   struct v3_sleep   v3sleep;
   //struct v3_message v3ack;
   struct v3_message v3nack;
//   struct v3_otadat  v3otadat;
} ;

extern union v3_message_UNION v3msgU;

extern union v3_message_UNION v3msgflashU;


#endif /* V3.h */
