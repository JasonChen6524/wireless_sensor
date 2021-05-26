/***************************************************************************//**
 * @file
 * @brief init_board.c
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#include "hal-config.h"
#else
#include "bspconfig.h"
#endif

#include "board_features.h"
#include "global.h"
#include "em_cmu.h"
#include "em_msc.h"
#include "V3.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
//#include "MAX14676E.h"
#include "retargetswo.h"
#include "mscflash.h"
#include "app.h"

extern void bpt_init(void);  // temporarily put here, should go elsewhere

#warning "WARNING: Custom boards contain no init code in initBoard. Please make sure you have created the init code needed for your board."
void initBoard(void)
{

  // Enable clock for USART0 (I2S0)
  CMU_ClockEnable(cmuClock_USART0, true);
    // Enable clock for USART2 (I2S1)
  CMU_ClockEnable(cmuClock_USART2, true);
  // Enable GPIO clock source
  CMU_ClockEnable(cmuClock_GPIO, true);
  // Place custom board initialization code here.
  CMU_ClockEnable(cmuClock_I2C0, true);
  
  flash_init(false); // processor flash, do not wipe unless never initialized (preserve serial number)
  initI2C();
  //max14676_init();
  initSPI();
  v3_init();  //keep after flash_init() and initSPI();
  initI2S(0);
  /* Initialize debug prints. Note: debug prints are off by default. See DEBUG_LEVEL in app.h */
  initLog();                                 // have to call the init before calling bpt_init()
  bpt_init(); // bio-sensor initilization

}


void initVcomEnable(void)
{
}
