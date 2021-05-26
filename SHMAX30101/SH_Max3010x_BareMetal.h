
/*******************************************************************************
 * Copyright (C) 2018 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *******************************************************************************
 */

#ifndef SH_Max3010x_BareMetal_H_
#define SH_Max3010x_BareMetal_H_

#include <stdint.h>
#include <stdio.h>

#include <stdint.h>
#include <stdio.h>
#include "app.h"

#define BPT_ALGO_VERSION_600


#define COMM_SUCCESS        0
#define COMM_GENERAL_ERROR  -1
#define COMM_INVALID_PARAM  -254
#define COMM_NOT_RECOGNIZED -255

#define DISABLE_AGC_USAGE 0
#define ENABLE_AGC_USAGE  1

#define BPT_CALIBRATION_TIMEOUT_SECONDS ((int) 70)
#define BPT_CALIBRATION_PROCESSDONE_VAL ((int) 100)


 #define CAL_DATA_SIZE_BYTES ((int) 512)

#define STATUS_OFFSET_BYTE ((int) 1)


enum bptExecMode{

	bptExecModeCalibration    = 1,
	bptExecModeEstimation     = 2,
};


/**
* @brief	Initialize Max30101 with default configuration
*
* @param[in]	agc_enabled - check whether to enable agc or not
*
* @return SS_STATUS byte
*/
int SH_Max3010x_default_init( const int algoExecutionMode, const int agc_usage );
int SH_Max3010x_algo_start( const int algoExecutionMode);                                     // Added by Jason Chen
int SH_Max3010x_algo_start_02( const int algoExecutionMode);                                  // Added by Jason Chen, 2021.03.04
int SH_Max3010x_algo_start_02_status( const int algoExecutionMode);                           // Added by Jason Chen, 2021.03.04
int SH_Max3010x_sensor_enable( void );                                                        // Added by Jason Chen, 2021.03.04
int SH_Max3010x_sensor_enable_status( void );                                                 // Added by Jason Chen, 2021.03.04

typedef int (*cmdExecFunc)( const char*); // typedef void (*cmdExecFunc)( const void*);

typedef struct {
	char const* cmdStr;
	cmdExecFunc execute;
	char const *help;
}cmd_interface_tp;


int command_help(const char *str);



/**
* @brief	gets the data format of reported whrm algo result sample , in form of struct fields with number of bits they are represented.
*
* @param[in] null_arg : NULL string, just to match the form of command table function pointer type
*
* @return 0
*/
int SH_Max3010x_get_bpt_dataformat(const char *null_arg);


/**
* @brief	sets the register value of ppg sensor (8614x is connected to ME11 SENSOR HUB for this demo but can be used for all sensors physically
*           connected to ME11 SENSOR HUB).
*
* @param[in] addr_value_args : byte string including command followed by register address in hex and register value in hex
*                              "set_reg ppgsensor 0xAB 0xCD" or "set_reg ppgsensor AB CD"
*
* @return 0x00 on success and prints on command console
*/
int SH_Max3010x_set_ppgreg(const char *addr_value_args);

/**
* @brief	gets the register value of ppg sensor (8614x is connected to ME11 SENSOR HUB for this demo but can be used for all sensors physically
*           connected to ME11 SENSOR HUB).
*
* @param[in] addr_arg: byte string including command followed by register address in hex
*                      "set_reg ppgsensor 0xAB" or "set_reg ppgsensor AB"
*
* @return 0x00 on success and prints register value on command console
*/
int SH_Max3010x_get_ppgreg(const char *addr_arg);

/**
* @brief	  initalizes and starts BPT mearurement for calibration data acquisition from ME11 SENSOR HUB.
*             It default initilizes datatype(to BOTH) input fifo lenght(to 5 , this is FIFO used by accel data from host)
*
* @param[in]  null_arg : NULL string, just to match the form of command table function pointer type
*
** @return 0x00 on success and prints status on command console
*/
int SH_Max3010x_measure_bpt_calibration (const char *null_arg);

/**
* @brief	  initalizes and starts BPT estimation from ME11 SENSOR HUB.
*             It default initilizes datatype(to BOTH) input fifo lenght(to 5 , this is FIFO used by accel data from host)
*
* @param[in]  null_arg : NULL string, just to match the form of command table function pointer type
*
** @return 0x00 on success and prints status on command console
*/
int SH_Max3010x_measure_bpt_estimation(const char *null_arg);

/**
* @brief	  eanbles/disables automatic gain control usage
*
* @param[in]  config : byte string including command followed by mode value in HEX uo to 1 hexadecimal digits.
*                         "set_cfg ppg_agc 0/1"  0:off 1: On
*
*@return  status info, 0x00 on success
*
**/
int SH_Max3010x_set_ppg_agcusage(const char *config);

/**
* @brief	  sets the  parameter for BPT algorithm
*
* @param[in]  config_arg : byte string including command followed by
*                          "set_cfg bpt med   "
*
* @return status info, 0x00 on success.
**/
int SH_Max3010x_set_bpt_med_config(const char *config_arg);

/**
* @brief	  sets the cal_index, systolic, diastolic parameter for BPT algorithm
*
* @param[in]  config_arg : byte string including command followed by
*                          "set_cfg bpt sys_dia   "
*
* @return status info, 0x00 on success.
**/
int SH_Max3010x_set_bpt_sys_dia_config(const char *config_arg);

/**
* @brief	  sets the  parameter for BPT algorithm
*
* @param[in]  config_arg : byte string including command followed by
*                          "set_cfg bpt sys_bp   "
*
* @return status info, 0x00 on success.
**/
int SH_Max3010x_set_bpt_sysbp_config(const char *config_arg);

/**
* @brief	  sets the  parameter for BPT algorithm
*
* @param[in]  config_arg : byte string including command followed by
*                          "set_cfg bpt dia_bp   "
*
* @return status info, 0x00 on success.
**/
int SH_Max3010x_set_bpt_diabp_config(const char *config_arg);

/**
* @brief	  returns the cal_index used by BPT algorithm
*
* @param[in]  null_arg : NULL string, just to match the form of command table function pointer type
              prints calibration data byte string to commnad console.
*
* @return status info, 0x00 on success.
**/
int SH_Max3010x_get_bpt_cal_index(const char *null_arg);

/**
* @brief	  sets the  cal_index for BPT algorithm

* @param[in]  null_arg : NULL string, just to match the form of command table function pointer type
*             prints date and time data to commnad console.
*
* @return status info, 0x00 on success.
**/
int SH_Max3010x_set_bpt_cal_index(const char *config_arg);


/**
* @brief	  returns the calibration data currently used by BPT algorithm
*
* @param[in]  null_arg : NULL string, just to match the form of command table function pointer type
              prints calibration data byte string to commnad console.
*
* @return status info, 0x00 on success.
**/

int SH_Max3010x_get_bpt_calibration(const char *null_arg);

/**
* @brief	  sets the  calibration data for BPT algorithm
*
* @param[in]  config_arg : byte string including command followed by calibration data of length: 624 x 1/2 bytes
*                          "set_cfg bpt cal_data XXXXXXXXXXXXXXXX..."
*
* @return status info, 0x00 on success.
**/
int SH_Max3010x_set_bpt_calibration(const char *config_arg);

/**
* @brief	  sets the  date and time for tracking
*
* @param[in]  time_arg : byte string including command followed by
*                          "set_cfg bpt cal_data XXXXXXXXXXXXXXXX..."
*
* @return status info, 0x00 on success.
**/
int SH_Max3010x_set_bpt_date_time(const char *config_arg);

/**
* @brief	  gets the  date and time data currently used by BPT algorithm
*
* @param[in]  null_arg : NULL string, just to match the form of command table function pointer type
*             prints date and time data to commnad console.
*
* @return status info, 0x00 on success.
**/
int SH_Max3010x_get_bpt_date_time(const char *null_arg);


/**
* @brief	   sets the non rest configuration for BPT algorithm
*
* @param[in]  time_arg : byte string including command followed by
*                        "set_cfg bpt time"
*
* @return status info, 0x00 on success.
**/
int SH_Max3010x_set_bpt_nonrest_config(const char *config_arg);


/**
* @brief	   sets the non rest configuration for BPT algorithm
*
* @param[in]  config_arg : byte string including command followed by
*                        "set_cfg bpt spo2_coefs "
*
* @return status info, 0x00 on success.
**/
int SH_Max3010x_set_bpt_spo2_coeffs(const char *config_arg);

/**
* @brief	   gets currenlty employed spo2 coefficients used by BPT algorithm
*
* @param[in]  null_arg : NULL string, just to match the form of command table function pointer type
*             prints spo2 coefficients to commnad console.
*
* @return status info, 0x00 on success.
**/
int SH_Max3010x_get_bpt_spo2_coeffs(const char *null_arg);


/**
* @brief	  demo specific fucntion to disable algorithm and PPG sensor
*
* @param[in]  null_arg : NULL string, just to match the form of command table function pointer type

*
* @return status info, 0x00 on success.
**/

int SH_Max3010x_stop(const char *null_arg);

/**
* @brief	Check the data stored in the Sensor Hub. Reads and prints
* 			the data if available
*
*/
void SH_Max3010x_data_report_execute(void);
void SH_Max3010x_data_report_execute02(void);

//FOR MAIN DEMO USAGE THIS WILL BE REPLACED!!
int ssx_set_algo_cfg(int algo_idx, int cfg_idx, uint8_t *cfg, int cfg_sz);



//#define NUMCMDS3010xBPT (21)

//extern const cmd_interface_tp CMDTABLE3010xBPT[];

/*DEMO RELATED DATA TO REPORT RESULTS FROM MAIN APPLICATION*/
extern uint8_t  bptMesurementProgress    ;

#endif
