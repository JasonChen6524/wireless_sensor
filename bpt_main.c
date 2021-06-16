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


/**********************************************************************************
 *
 *  Desc: Example Code to get algorithm estimation results of Blood Pressure from sensor hub and display on Command Console
 *        Example starts in Example Measurement Mode.
 *
 *        Example,
 *
 *        1. In Command Mode
 *                   Executes MAx3010xBPT Command Table commands from user.
 *
 *        2. In example User Mode
 *                   1. Sets Coefficients for calibration med, rest, time, spo2 coefs, sys_bp , dia_bp
 *                   2. Starts calibration
 *                   	        1. initialize algorithm config struct enabled
 *                              2. enable data type to both raw sensor and algorithm data
 *                              3. set fifo threshold for mfio event frequency
 *                              5. enable sensor to acquire ppg data
 *                              6. disables AGC algorithm
 *                              7. enable BPT algorithm in calibration mode
 *
 *                   3. get BPT calibration for USER
 *                   4. Sets Coefficients for estimation med, rest, time, spo2 coefs
 *                   5. Sets calibration data before each estimation measurement
 *                   6. Starts BPT Estimation for USER
 *                   	        1. initialize algorithm config struct enabled
 *                              2. enable data type to both raw sensor and algorithm data
 *                              3. set fifo threshold for mfio event frequency
 *                              5. enable sensor to acquire ppg data
 *                              6. enables/disables AGC algorithm
 *                              7. enable BPT algorithm in estimation mode
 *
 *                  7. Example calls SH_Max3010x_data_report_execute() which
 *				                1. calls SH API's sh_ss_execute_once() function which:
 *                                 writes sensor hub's report fifo content (sensor/algorithm data samples) to a buffer(1).
 *                              2. Parses buffer(1) data to extract numeric sensor and algorithm samples according to enabled algorithms.
 * 			                      look:  bpt_data_rx() , max3010x_data_rx() and sample structs defined within SH_Max3010x_BareMetal.cpp
 *
 *        3. In Booloader Mode
 *                 Executes Bootloader Command Table commands from user. Look at Bootloader Python Script for PC side example.
 *
 *
 *
 *
 ***********************************************************************************/

#include "Global.h"

#include <stdbool.h>
#include <stdio.h>
//#include "app.h"
#include "V3.h"
#include "sl_sleeptimer.h"

#include "SSMAX30101Comm.h"
#include "DSInterface.h"
#include "SSInterface.h"
#include "utils.h"

demo_appstate_t appState  = ST_EXAMPLEUSER_IDLE01;//ST_COMMAND_MODE;

sl_sleeptimer_timer_handle_t bpt_timer;
static uint16_t bpt_second_count = 0;
void bpt_timer_callback(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  //Code executed when the timer expire.
	bpt_second_count++;
  //printLog("BPT Timer: %d\r\n", bpt_second_count);
}

uint16_t calibrationTimer_read(void)
{
	return bpt_second_count;
}

void calibrationTimer_reset(void)
{
	bpt_second_count = 0;
}

void calibrationTimer_start(void)
{
	sl_status_t status;

	uint32_t timer_timeout = sl_sleeptimer_ms_to_tick(10);                // 10 ms
	bpt_second_count = 0;

	status = sl_sleeptimer_start_periodic_timer(&bpt_timer,
	                                            timer_timeout,
	                                            bpt_timer_callback,
	                                            (void *)NULL,
	                                            0,
	                                            0);
	if(status != SL_STATUS_OK) {
	    printLog("Timer not started.\r\n");
	}
}
void calibrationTimer_stop(void)
{
	sl_sleeptimer_stop_timer(&bpt_timer);
}


const char *fw_version;
uint32_t count_tick_last = 0;

const char* myget_ss_fw_version(int bootldr)
{
    uint8_t cmd_bytes[2];
    uint8_t rxbuf[4];

    static char fw_version[32] = "Unknown SS SENSORHUB";
	//int bootldr = ss_in_bootldr_mode();

	if (bootldr > 0) {
		cmd_bytes[0] = SS_FAM_R_BOOTLOADER;
		cmd_bytes[1] = SS_CMDIDX_BOOTFWVERSION;
	} else if (bootldr == 0) {
		cmd_bytes[0] = SS_FAM_R_IDENTITY;
		cmd_bytes[1] = SS_CMDIDX_FWVERSION;
	} else {
		return "Unknown FW";
	}

    SS_STATUS status = read_cmd(
             &cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
             0, 0,
             &rxbuf[0], ARRAY_SIZE(rxbuf), SS_DEFAULT_CMD_SLEEP_MS);

    if (status == SS_SUCCESS) {
        snprintf(fw_version, sizeof(fw_version),
            "%d.%d.%d", rxbuf[1], rxbuf[2], rxbuf[3]);
		printLog("fw_version:%s\r\n", fw_version);
    }

    return &fw_version[0];
}

const char* myget_ss_algo_version(int bootldr)
{
    uint8_t cmd_bytes[3];
    uint8_t rxbuf[4];

    static char algo_version[64] = "Unknown SS HUB ALGORITHMS";
	//int bootldr = ss_in_bootldr_mode();

	if (bootldr > 0) {
		cmd_bytes[0] = SS_FAM_R_BOOTLOADER;
		cmd_bytes[1] = SS_CMDIDX_BOOTFWVERSION;
		cmd_bytes[2] = 0;
	} else if (bootldr == 0) {
		cmd_bytes[0] = SS_FAM_R_IDENTITY;
		cmd_bytes[1] = SS_CMDIDX_ALGOVER;
		cmd_bytes[2] = SS_CMDIDX_AVAILSENSORS;
	} else {
		return "Unknown ALGO";
	}

    SS_STATUS status = read_cmd(
             &cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
             0, 0,
             &rxbuf[0], ARRAY_SIZE(rxbuf), SS_DEFAULT_CMD_SLEEP_MS);

    if (status == SS_SUCCESS) {
        snprintf(algo_version, sizeof(algo_version),
            "%d.%d.%d", rxbuf[1], rxbuf[2], rxbuf[3]);
		printLog("algo_version:%s\r\n", fw_version);
    }

    return &algo_version[0];
}

int bio_mode = -1;
void bpt_init(void)
{
	wait_ms(2000);
	ss_init_hwcomm_interface();
	ss_disable_irq();
	ss_clear_mfio_event_flag();
	ss_enable_irq();

	bio_mode = ss_in_bootldr_mode();//sh_checkif_bootldr_mode();

	fw_version = myget_ss_fw_version(bio_mode);
	printLog("Hub FW Version:%s, Line=%d\r\n",fw_version, __LINE__);
	fw_version = myget_ss_algo_version(bio_mode);
	printLog("Hub algo Version:%s, Line=%d\r\n",fw_version, __LINE__);
}

void bpt_main(void)
{
	//static int measurement_count = 0;
    //while(1)
    {
		switch(appState)
		{
			case ST_COMMAND_MODE:
			{
				//appState = ST_EXAMPLEUSER_CALIBRATION_SETTINGS;

				bio_mode = ss_in_bootldr_mode();//sh_checkif_bootldr_mode();

				//wait_ms(500);
				fw_version = myget_ss_fw_version(bio_mode);
				printLog("Hub FW Version:%s, Line=%d\r\n",fw_version, __LINE__);
				//wait_ms(500);
				fw_version = myget_ss_algo_version(bio_mode);
				printLog("Hub algo Version:%s, Line=%d\r\n",fw_version, __LINE__);
				if(bio_mode == 0)
				{
					//appState = ST_EXAMPLEUSER_CALIBRATION_SETTINGS;
					appState = ST_EXAMPLEUSER_IDLE;
					printDebug("\r\nStarting Calibrating...............\r\n\r\n");
				}
				else if(bio_mode > 0)
				{
					printLog("Hub bootloader........., Line = %d\r\n", __LINE__);
				}
				else
					printLog("Unknown Mode........., Line = %d\r\n", __LINE__);
			}
			break;

			case ST_EXAMPLEUSER_IDLE:
			{
				  set_fw_platform(get_ss_platform_name());
				  set_fw_version(get_ss_fw_version());
#if 1
				  printLog("Sensor FW   Version:%s\r\n",     get_ss_fw_version());
				  printLog("Platform Name:      %s\r\n",     get_ss_platform_name());
				  printLog("Part Name:          %s\r\n",     ssMAX30101.get_part_name());
				  printLog("Sensor Algo Version:%s\r\n",     ssMAX30101.get_algo_ver());
#endif

				  //add_sensor_comm((SensorComm *)&ssMAX30101);
				  //add_sensor_comm((SensorComm *)&ssBoot);
				  //add_sensor_comm((SensorComm *)&ssGenericCmd);
				  appState = ST_EXAMPLEUSER_IDLE01;
			}
			break;

			case ST_EXAMPLEUSER_IDLE01:
			{
			}
			break;
		}

		//wait_ms(2);
		//SH_Max3010x_data_report_execute02();
	}
}

void bpt_main_reset(void)
{
#if 0
	//curren_tick1 = sl_sleeptimer_get_tick_count();
	printDebug("TIMEOUT/FAULT CONDITION OCCURED FOR EXAMPLE MEASUREMENT. \r\n");
  //is_example_measurement_active =  false;
	calibrationTimer_stop();
	SH_Max3010x_stop(0);
	appState = ST_EXAMPLEUSER_DELAY_COUNT;                     //Jason //ST_COMMAND_MODE;
	delay_ms_count = 0;
	state_flag = 1;
#endif
}


