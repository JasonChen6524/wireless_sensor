/***************************************************************************
 *
 *            Copyright (c) 2012 by Artafelx INC.
 *
 * This software is copyrighted by and is the sole property of
 * Artaflex INC.  All rights, title, ownership, or other interests
 * in the software remain the property of Artaflex INC.  This
 * software may only be used in accordance with the corresponding
 * license agreement.  Any unauthorized use, duplication, transmission,
 * distribution, or disclosure of this software is expressly forbidden.
 *
 * This Copyright notice may not be removed or modified without prior
 * written consent of Artaflex INC.
 *
 * Artaflex INC reserves the right to modify this software without notice.
 *
 * Artaflex INC.
 * 96 Steelcase Road West,
 * Markham, ON L3R 3T9
 * Canada
 *
 * Tel:   (905) 470-0109
  * http:  www.artaflex.com

 *
 ***************************************************************************/
#ifndef _MAX14676_H_
#define _MAX14676_H_

#define MAX14676E_ADR50	0x50 //PMIC 8 bit (shifted) I2C address
#define MAX14676E_ADR6C	0x6C //PMIC fuel guage 8 bit I2C address

//MAX initialization values
#define LEDSTRT 0x24
#define MAX_LEDN_CFG 0x20
#define MAX_BST_ON 0x49 // Enable BOOST at +14V
#define MAX_BST_OFF  0x09  // Turn off BOOST supply
#define MAX_CDET_INIT 0x84 // CDETCNTLB reg init - Charger Auto Restart
#define MAX_VSET_INIT 0x00


//MAX 0x50 addressed registers
#define MAX_CHIP_ID			0x00
#define MAX_STATUSA			0x02
#define MAX_STATUSB			0x03
#define MAX_ILIMCNTL_ADDR 	0x0A
#define MAX_CHGCNTLA       	0x0B
#define MAX_CHGCNTLB       	0x0C
#define MAX_CHGTMR			0x0D
#define MAX_CHGVSET			0x0E
#define MAX_PCHGCNTL        0x10
#define MAX_CDETCNTLB		0x11
#define MAX_LDOCFG         	0x16
#define MAX_BST_CFG			0x19
#define MAX_LED_CFG			0x1A
#define MAX_LED0_RED	 	0x1B
#define MAX_LED1_GRN		0x1C
#define MAX_LED2_BLU		0x1D
#define MAX_PWR_CFG_ADDR 	0x1E

//MAX register masks
#define MAX_STATB_CHGIN 0x08  //Charge is connected

//MAX 0x6C addressed registers
#define MAX_VCELL	0x02
#define MAX_SOC		0x04
#define MAX_CRATE	0x16

extern bool max14676_init(void);

extern void max14676_LED(struct rgbcolor ledset); //255 max rgb values

//void max14676_process(void);

extern U8 max14676_GetBatteryCharge(void);
extern U16 max14676_GetBatteryChargeVoltage( void);
extern U16 max14676_GetBatteryCRATE( void);
extern U8 max14676_GetStatusB(void);
extern U8 max14676_GetStatusA(void);
extern void max14676_ChargerOn(void);
extern void max14676_Boost(U8 state);
extern void max14676_poweroff(void);


 


#endif
