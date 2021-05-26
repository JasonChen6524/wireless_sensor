
#ifndef I2C_H            // Guard against multiple inclusion
#define I2C_H

#include "Global.h"

// MAX30208 temperatuer sensor defines
//#define MAX30208A_ADR 0xA0	// Cabled Temp sensor First address
#define MAX30208A_ADR 0xA4	// Cabled Temp sensor First address
#define MAX30208B_ADR 0xA0	// Second Temp sensor TBD address

#define MAX30208_ID	0xFF	// Part Identifier
#define MAX30208_SETUP	0x14	// part Setup/ initiate register TEMP_SENSOR_SETUP
#define MAX30208_CFG2	0x0A	// FIFO confiuration 2 - FIFO CONFIGURATION 2
#define MAX30208_DATA 	0x08	// FIFO Data register FIFO_DATA
#define MAX30208_COUNT	0x07	// FIFO count FIFO_DATA_COUNT
#define MAX30208_STATUS 0x00	// Part Staus register
#define MAX30208_RESET  0x0C	// Write 0x01 to reset the chip into default status,                                     //Jason Chen, 2021.04.22
#define MAX30208_STATUS_RDY 0x01	// Status bit sample ready mask
#define MAX30208_SETUP_GO	0xC1	// Value to write to initiate a conversion
#define MAX30208_ID_VAL	0x30	// Value that should be in register MAX30208_ID
#define MAX30208_CFG2_FLUSH 0x10	// Value to write to CFG2 to flush FIFO

#define MAX30208_WAIT	0xFFFF	// temperature return value when present, waiting on first conversion
#define MAX30208_ERROR	0xDEAD	// temperature sensor does not respond

//structure for feedback player, buzzer and haptic
struct feedback // Hz, buz dur mS, hap dur mS
{
   uint16_t freq;
   uint8_t buztime;
   uint8_t haptime;
};

struct fbplay
{
	uint8_t num;	// what is playing
	uint8_t size;	// number of color entries
	uint8_t pos;	// LED color position to play
	uint8_t repeat; // times to repeat , 0xFF = forever
	struct feedback *seq;  // pointer to array of feedback elements - Hz, buz dur mS, hap dur mS
};

extern struct fbplay fbseqset;

#define FBPLAY_INIT                       \
{                                         \
   FBPOWERON,                             \
   sizeof(fbturnon)/sizeof(fbturnon[0]),  \
   0,                                     \
   1,                                     \
   (struct feedback *) fbturnon           \
};

//Feedback sequence defines
#define FBFOREVER 0XFF
#define MAXFBSEQ 4
#define FBNONE    0
#define FBPOWERON 1
#define FBPOWEROFF 2
#define FBSHORTUP 3
#define FBSHORTDN 4

// led color struture
struct rgbcolor
{
	uint8_t red;
	uint8_t grn;
	uint8_t blu;
};

struct ledplay
{
	uint8_t num;	// what is playing
	uint8_t size;	// number of color entries
	uint8_t pos;	// LED color position to play
	uint8_t repeat; // times to repeat , 0xFF = forever
	struct rgbcolor * seq;  // pointer to an array of rgbcolor red, green blue entry, 0- 255
};
extern struct ledplay ledseqset;

#define LEDPLAY_INIT                       \
{                                          \
   LEDPOWERON,                             \
   sizeof(ledturnon)/sizeof(ledturnon[0]), \
   0,                                      \
   1,                                      \
   (struct rgbcolor*) ledturnon            \
};

// LED sequencer defines - see led.h
#define LEDFOREVER 0XFF
#define MAXLEDSEQ 10
#define LEDBLACK 0  // leds off, no sequencing
#define LEDPOWERON 1
#define LEDYELFLASH 2  // yellow flashing
#define LEDYELPAUSE 3  // Slow yellow flashing Paused
#define LEDBLECON 4 // blue flashing
#define LEDORGFLASH 5
#define LEDPOWEROFF 6 // Power off sequence
#define LEDCHRG00 7
#define LEDCHRG25 8
#define LEDCHRG50 9
#define LEDCHRG95 10



// color init defs LS 3 bits are not used
#define LED_OFF {0,0,0}
#define DIM_RED	{0x10,0,0}
#define DIM_GRN	{0,0x10,0}
#define DIM_BLU	{0,0,0x10}
#define DIM_WHT	{0x10,0x10,0x10}

#define HALF_RED	{0x80,0,0}
#define HALF_GRN	{0,0x80,0}
#define HALF_BLU	{0,0,0x80}
#define HALF_WHT	{0x80,0x80,0x80}

#define FULL_RED	{0xFF,0,0}
#define FULL_GRN	{0,0xFF,0}
#define FULL_BLU	{0,0,0xFF}
#define FULL_WHT	{0xFF,0xFF,0xFF}

#define LED_INIT	FULL_BLU

//IO expander I2C address (8 bit)
#define IO_EXP_ADR 0x37

#define	IO_EXP_CMD_SIZE 4

// I/O expander commands
#define CMD_VER         0x01  // return FW version
#define CMD_SPK         0x02  // control Piezo beeps
#define CMD_HAP         0x03  // control Haptic vibration timing
#define CMD_GET         0x04  // Get bits of I/O
#define CMD_SET         0x05  // SET I/O bits 
#define CMD_LAST        0x06

//See V3 board Schematics
// Supported CMD_SET bits. MFIO is open collector, high = input.
//  MUX = 0 selects codec sine generators
//  D7 | D6|   D5  | D4|  D3  |  D2 |   D1 | D0 |
//   X | X | RESET | X | MFIO | MUX | EN5V | X  |
//   X | X |   0   | X |  OC  |  0  |   0  | X  |  //POR values OC = input

// CMD_GET bit mappings
//  D7 | D6   | D5  |  D4  |  D3  |  D2 |  D1 | D0 |
//P_INT|P_ALRT|P_BTN|CON_S1|CON_S0|RESET| BIO |MFIO|  // BIO sensor bit is inserted by the V3


#if 0
#define CMD_GET_MFIO    0x06  // Get bits of MFIO I/O
#define CMD_SET_DIR     0x07  // Set RESET, MFIO Input/Output
#define CMD_IRQ_ENABLE  0x08  // MFIO inrrupt Enabled
#define CMD_IRQ_DISABLE 0x09  // MFIO inrrupt Disabled
#define CMD_LAST        0x0A
#endif

// Expander responses
#define CMD_ACK 0xAA
#define CMD_ERR 0x55

#define IOTICSPERSEC 100  // I/O expander tick conversion rate

//speaker params
#define SPK_FREQ(HZ) (1000000/(HZ*2))
#define SPK_TIME(MS) (MS/10)

// Haptic parameters
#define HAP_TIME	SPK_TIME

// SET I/O PIN values
#define SET_PIN_EN5V 0x02
#define EN5V_HIGH	 0x02
#define EN5V_LOW     0x00
#define MFIO_HIGH	0x08
#define MFIO_LOW	0x00
#define RESET_HIGH  0x20
#define RESET_LOW 	0x00


#define RESET_BIT	              0x20
#define MFIO_BIT	              0x08

#if 0
#define OUTPUT_HIGH	              1
#define OUTPUT_LOW                0
#define BIT_INPUT                 1
#define BIT_OUTPUT	              0
#endif


extern void ExpSpeakerOn(U16 freq, U8 time);  // freq in HZ, time in mS
extern void ExpHapOn(U8 time);// time in mS
extern int I2C_READ(U8 addr, U8 reg, U8 *data, U16 len);
extern int I2C_WRITE(U8 addr, U8 reg, U8 *data, U16 len);
extern int I2C_CMD_READ(U8 addr, U8 *data, U16 len);
extern int I2C_CMD_IO(U8 addr, U8 *data);
extern void initI2C(void);
extern U16 Temperature(U8 addr);
extern void ShutDown(void);
extern U8 ExpGetVer(void);
extern U8 ExpGetPins(void);
extern U8 ExpSetPins(U8 bits);
extern void ledseq(void);
extern void fbseq(void);
extern void ledseq_set(U8 num, U8 repeat);
extern void fbseq_set(U8 num, U8 repeat);
extern void MAXBoostOn(void);
extern void MAXBoostOff(void);
extern void Temperature_reset(void);                                                                           //Jason Chen, 2021.04.22
//extern U8 ExpGetMFIO(void);                                                     //Jason
//extern U8 ExpSetPins_2(U8 bits);                                                //Jason
//extern U8 ExpResetPins_2(U8 bits);                                              //Jason
//extern U8 ExpSetPinsOutput(U8 bits);                                            //Jason
//extern U8 ExpSetPinsInput(U8 bits);                                             //Jason
//extern U8 ExpSetIRQ_Enable(void);                                               //Jason
//extern U8 ExpSetIRQ_Disable(void);                                              //Jason

extern int m_i2cBus_write(U8 addr, U8 *cmd_bytes, int cmd_bytes_len, bool flag);//Jason
extern int m_i2cBus_read(U8 addr, U8 *data, U16 len);                           //Jason

//extern const struct rgbcolor led_00[];
//extern const struct rgbcolor led_10[];
//extern const struct rgbcolor led_30[];
//extern const struct rgbcolor led_50[];
//extern const struct rgbcolor led_75[];
//extern const struct rgbcolor led_95[];



#endif                      // Avoid multiple inclusion

/*************************** End of file ****************************/
