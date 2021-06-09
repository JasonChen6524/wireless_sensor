/***************************************************************************//**
 * @brief app.h
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

#ifndef APP_H_
#define APP_H_

#include <stdbool.h>
#include "gecko_configuration.h"

/* DEBUG_LEVEL is used to enable/disable debug prints. Set DEBUG_LEVEL to 1 to enable debug prints */
#define DEBUG_LEVEL 1

/* Set this value to 1 if you want to disable deep sleep completely */
#define DISABLE_SLEEP 1

#if DEBUG_LEVEL
#include "retargetswo.h"
#include <stdio.h>
#endif

#if DEBUG_LEVEL
	#define initLog()     RETARGET_SwoInit();
	//#define flushLog()    RETARGET_SerialFlush()
	#define printLog(...) printf(__VA_ARGS__)
	#if 0
	  #define printDebug(...) printf(__VA_ARGS__)
	#else
	  #define printDebug(...)
	#endif

	#if 0
	  #define printBleDebug(...) printf(__VA_ARGS__)
	#else
	  #define printBleDebug(...)
	#endif

#else
	#define initLog()
	#define flushLog()
	#define printLog(...)

	#define printDebug(...)
	#define printBleDebug(...)
#endif

typedef const char* (*get_type_callback)(void);
typedef uint8_t (*is_visible_callback)(void);
typedef int (*get_part_info_callback)(uint8_t*, uint8_t*);
//typedef uint8_t (*is_enabled_callback)(void);
typedef void (*SensorComm_Set_Ble_Status_callback)(bool, uint8_t);
typedef const char* (*get_part_name_callback)(void);
typedef const char* (*get_algo_ver_callback)(void);
typedef void (*stop_callback) (void);

typedef uint8_t (*parse_command_callback)(const char*);
typedef int  (*data_report_execute_callback)(char*, int);


#define DS_MAX_NUM_SENSORCOMMS	8

/* Main application */
void appMain(gecko_configuration_t *pconfig);

bool BLE_Interface_Exists(void);
int BLE_AddtoQueue(uint8_t *data_transfer, int32_t buf_size, int32_t data_size, int32_t line);
void BLE_reset_queue(void);

#endif
