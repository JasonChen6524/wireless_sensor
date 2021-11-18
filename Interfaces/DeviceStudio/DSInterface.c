/*
 * DSInterface.c
 *
 *  Created on: May 28, 2021
 *      Author: jason Chen Local
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "version.h"
#include "app.h"
#include "Peripherals.h"
#include "DSInterface.h"
#include "assert.h"
//#include "spp_utils.h"
#include "sl_sleeptimer.h"

#include "SSMAX30101Comm.h"
#include "SSBootloaderComm.h"
#include "SSGenericCmd.h"
#include "utils.h"


static const char *glbl_cmds[] = {
	"stop",
	"get_device_info",
	"silent_mode 0",
	"silent_mode 1",
	"pause 0",
	"pause 1",
	"enable console",
	"disable console",
};

typedef enum {
	stop=0,
	get_device_info,
	silent0_mode,
	silent1_mode,
	pause0_mode,
	pause1_mode,
	enable_console,
	disable_console,
	NUM_CMDS,
} glbl_cmd_state;

int num_sensors;
SensorComm sensor_list_t[DS_MAX_NUM_SENSORCOMMS];
SensorComm* sensor_list = (SensorComm*)(&sensor_list_t[0]);

//bool calibration_success = false;
volatile uint8_t ds_console_interface_exists_;
//volatile uint8_t ds_ble_interface_exists_;

const char* platform_name;
const char* firmware_version;

#define CONSOLE_STR_BUF_SZ                    (2048)
char cmd_str[CONSOLE_STR_BUF_SZ];

int cmd_idx;
bool silent_mode;
bool pause_mode;

void DSInterface_init(void)
{
	cmd_idx = 0;
	silent_mode = false;
	pause_mode = false;

	memset(&sensor_list[0], 0, DS_MAX_NUM_SENSORCOMMS * sizeof(SensorComm*));
	num_sensors = 0;

	//m_USB = USB;

	ds_console_interface_exists_ = false;


}

void SensorComm_create(void)
{
	SSMAX30101Comm_init();
	SSBootloaderComm_init();
	SSGenericCmd_int();
	int i = 0;
		sensor_list_t[i].get_type                  = (get_type_callback)ssMAX30101.get_type;
		sensor_list_t[i].get_part_info             = (get_part_info_callback)ssMAX30101.get_part_info;     //
		sensor_list_t[i].is_visible                = (is_visible_callback)ssMAX30101.is_visible;
		//sensor_list_t[i].is_enabled                 = (is_enabled_callback)ssMAX30101.is_enabled;
		sensor_list_t[i].SensorComm_Set_Ble_Status = (SensorComm_Set_Ble_Status_callback)ssMAX30101.SensorComm_Set_Ble_Status;

		sensor_list_t[i].get_part_name             = (get_part_name_callback)ssMAX30101.get_part_name;
		sensor_list_t[i].get_algo_ver              = (get_algo_ver_callback)ssMAX30101.get_algo_ver;
		sensor_list_t[i].stop                      = (stop_callback)ssMAX30101.stop;
		sensor_list_t[i].parse_command             = (parse_command_callback)ssMAX30101.parse_command;
		sensor_list_t[i].data_report_execute       = (data_report_execute_callback)ssMAX30101.data_report_execute;

	i = 1;
		sensor_list_t[i].get_type                  = (get_type_callback)ssGenericCmd.get_type;
		sensor_list_t[i].is_visible                = (is_visible_callback)ssGenericCmd.is_visible;
	  //sensor_list_t[i].is_enable                 = (is_enable_callback)&is_enabled_t;
	  //sensor_list_t[i].get_part_info             = (get_part_info_callback)ssGenericCmd.get_part_info;
	  //sensor_list_t[i].SensorComm_Set_Ble_Status = (SensorComm_Set_Ble_Status_callback)&SensorComm_Set_Ble_Status_t;

	  //sensor_list_t[i].get_part_name             = (get_part_name_callback)&get_part_name_t;
	  //sensor_list_t[i].get_algo_ver              = (get_algo_ver_callback)&get_algo_ver_t;
		sensor_list_t[i].stop                      = (stop_callback)ssGenericCmd.stop;
		sensor_list_t[i].parse_command             = (parse_command_callback)ssGenericCmd.parse_command;
	  //sensor_list_t[i].data_report_execute       = (data_report_execute_callback)&data_report_execute_t;
		sensor_list_t[i].data_report_execute       = (data_report_execute_callback)ssGenericCmd.data_report_execute;

	i = 2;
		sensor_list_t[i].get_type                  = (get_type_callback)ssBoot.get_type;
		sensor_list_t[i].is_visible                = (is_visible_callback)ssBoot.is_visible;
	  //sensor_list_t[i].is_enable                 = (is_enable_callback)ssBoot&is_enabled_t;
	  //sensor_list_t[i].get_part_info             = (get_part_info_callback)&get_part_info_t;
	  //sensor_list_t[i].SensorComm_Set_Ble_Status = (SensorComm_Set_Ble_Status_callback)&SensorComm_Set_Ble_Status_t;

	  //sensor_list_t[i].get_part_name             = (get_part_name_callback)&get_part_name_t;
	  //sensor_list_t[i].get_algo_ver              = (get_algo_ver_callback)&get_algo_ver_t;
		sensor_list_t[i].stop                      = (stop_callback)ssBoot.stop;
		sensor_list_t[i].parse_command             = (parse_command_callback)ssBoot.parse_command;
	  //sensor_list_t[i].data_report_execute       = (data_report_execute_callback)&data_report_execute_t;
		sensor_list_t[i].data_report_execute       = (data_report_execute_callback)ssBoot.data_report_execute;


	num_sensors = i + 1;
}


static void DSInterface_parse_command(void)
{
#if 1
	int i;
	glbl_cmd_state cmd;
	char charbuf[512];
	int data_len = 0;
	int ret;
	bool parsed_cmd = true;

	//If top level command, then handle it
	for (i = 0; i < NUM_CMDS; i++) {
		if (starts_with(&cmd_str[0], glbl_cmds[i])) {
			cmd = (glbl_cmd_state)i;

			switch (cmd) {
				case (stop): {
					for (int i = 0; i < num_sensors; i++) {
						sensor_list[i].stop();
					}
					data_len += snprintf(charbuf, sizeof(charbuf) - 1,
							"\r\n%s err=0\r\n", cmd_str);
				} break;
				case (get_device_info): {
					data_len = snprintf(charbuf, sizeof(charbuf),
						"\r\n%s platform=%s firmware_ver=%s sensors=", cmd_str, platform_name, FIRMWARE_VERSION);

					//Add list of sensors
					for (int i = 0; i < num_sensors; i++) {
						if (sensor_list[i].is_visible()) {
							data_len += snprintf(charbuf + data_len, sizeof(charbuf) - data_len,
											"%s", sensor_list[i].get_type());
							if (i < (num_sensors - 1))
								data_len += snprintf(charbuf + data_len, sizeof(charbuf) - data_len, ",");
						}
					}


					for (int i = 0; i < num_sensors; i++) {
						//SensorComm *s = &sensor_list[i];
						if (!sensor_list[i].is_visible())
							continue;

						//Add algo_ver
						data_len += snprintf(charbuf + data_len, sizeof(charbuf) - data_len,
											" algo_ver_%s=%s", sensor_list[i].get_type(), sensor_list[i].get_algo_ver());

						//Add part name
						data_len += snprintf(charbuf + data_len, sizeof(charbuf) - data_len,
											" part_name_%s=%s", sensor_list[i].get_type(), sensor_list[i].get_part_name());

						uint8_t part_id, part_rev;
						ret = sensor_list[i].get_part_info(&part_id, &part_rev);

						if (ret == 0) {
							//Add part id
							data_len += snprintf(charbuf + data_len, sizeof(charbuf) - data_len,
											" part_id_%s=%d", sensor_list[i].get_type(), part_id);

							//Add rev id
							data_len += snprintf(charbuf + data_len, sizeof(charbuf) - data_len,
											" part_rev_%s=%d", sensor_list[i].get_type(), part_rev);
						}
					}

					if(firmware_version){
						data_len += snprintf(charbuf + data_len, sizeof(charbuf) - data_len, " hub_firm_ver=%s", firmware_version);
					}

					data_len += snprintf(charbuf + data_len, sizeof(charbuf) - data_len, " err=0\r\n");

				} break;
				case (silent0_mode): {
					silent_mode = false;
					data_len += snprintf(charbuf, sizeof(charbuf) - 1,
							"\r\n%s err=0\r\n", cmd_str);
				} break;
				case (silent1_mode): {
					silent_mode = true;
					data_len += snprintf(charbuf, sizeof(charbuf) - 1,
							"\r\n%s err=0\r\n", cmd_str);
				} break;
				case (pause0_mode): {
					pause_mode = false;
					data_len += snprintf(charbuf, sizeof(charbuf) - 1,
							"\r\n%s err=0\r\n", cmd_str);
				} break;
				case (pause1_mode): {
					pause_mode = true;
					data_len += snprintf(charbuf, sizeof(charbuf) - 1,
							"\r\n%s err=0\r\n", cmd_str);
				} break;
				case (enable_console): {
					ds_console_interface_exists_ = true;
					data_len += snprintf(charbuf, sizeof(charbuf) - 1,
							"\r\n%s err=0\r\n", cmd_str);
				} break;
				case (disable_console): {
					ds_console_interface_exists_ = false;
					data_len += snprintf(charbuf, sizeof(charbuf) - 1,
							"\r\n%s err=0\r\n", cmd_str);
				} break;
				default:
					parsed_cmd = false;
					break;
			}

			if (parsed_cmd) {
				//m_USB->printf(charbuf);                                                        // Commented by Jason
#if 1//ENABLE_BLE
				if (BLE_Interface_Exists()) {
					BLE_AddtoQueue((uint8_t *)charbuf, (int32_t)sizeof(charbuf), data_len, __LINE__);
				}
#endif
                charbuf[data_len - 1] = 0;                                                        // Added by Jason
                charbuf[data_len - 2] = 0;                                                        // Added by Jason
                charbuf[1] = '\r';                                                                // Added by Jason
                printLog(&charbuf[1]);                                                             // added by Jason
                printLog(", DSI:Line=%d\r\n\r\n", __LINE__);                                       // Added by Jason
			}

			return;
		}
	}

	//Loop through each sensor in sensorList
	//If sensor.get_type() is contained in cmd_str, pass cmd_str to that sensor's parser
	for (int i = 0; i < num_sensors; i++) {
		//printLog("DSI cmd[%d]: %s, %s line:%d\r\n", i, cmd_str, sensor_list[i].get_type(), __LINE__);                                  // Added by Jason
		if (strstr(&cmd_str[0], sensor_list[i].get_type())) {
			//printLog("DSI cmd[%d]: %s, %s line:%d\r\n", i, cmd_str, sensor_list[i].get_type(), __LINE__);                              // Added by Jason
			if (sensor_list[i].parse_command(cmd_str))
            {
                //printLog("DSI cmd[%d]: %s, %s line:%d\r\n", i, cmd_str, sensor_list[i].get_type(), __LINE__);                          // Added by Jason  read ppg 9
				return;
            }
			break;
		}
	}
	//printLog("num_sensors = %d, Line:%d\r\n", num_sensors, __LINE__);                                                                 // Added by Jason
	//If we still haven't found a way to parse the command,
	//send it to every sensor until one handles it
	for (int i = 0; i < num_sensors; i++) {
		if (sensor_list[i].parse_command(cmd_str))
		{
			//printLog("DSI cmd[%d]: %s, %s line:%d\r\n", i, cmd_str, sensor_list[i].get_type(), __LINE__);                                  // Added by Jason for Auth
			return;
		}
	}


	//No one could handle the command, print cmd not recognized string
	data_len += snprintf(charbuf, sizeof(charbuf) - 1,
			"\r\n%s err=-255\r\n", cmd_str);
  //m_USB->printf(charbuf);                                                           // Commented by Jason
#if 1//ENABLE_BLE
	if (BLE_Interface_Exists()) {
		BLE_AddtoQueue((uint8_t *)charbuf, (int32_t)sizeof(charbuf), data_len, __LINE__);
	}
#endif
    charbuf[data_len - 1] = 0;                                                        // Added by Jason
    charbuf[data_len - 2] = 0;                                                        // Added by Jason
    charbuf[1] = '\r';                                                                // Added by Jason
    printLog(&charbuf[2]);                                                             // Added by Jason
    printLog(", DSI:Line=%d\r\n\r\n", __LINE__);                                       // Added by Jason
#endif
}

char cmd_str_log[54 + 23 + 1];
void DSInterface_BuildCommand(char ch)
{
	static int count_c = 0;
	//if (!this->silent_mode) /* BUG: POTENTIAL BUG, what uart port to echo, not only console */
	count_c++;
	if(count_c < (54 + 23 - 1))
	{
		//printLog("%c", ch);
	}

	if (ch == 0x00) {
		pr_err("Ignored char 0x00");
		return;
	}

	if ((ch == '\n') || (ch == '\r')) {
		if (cmd_idx < (int)CONSOLE_STR_BUF_SZ)
           cmd_str[cmd_idx++] = '\0';
		memcpy( &cmd_str_log[0], cmd_str , 54 + 23);
		if(count_c >= (54 + 23))
		{
			cmd_str_log[54 + 23] = '\0';
			printLog("%s....\r\n", cmd_str_log);
		}
		else
			printLog("%s\r\n", cmd_str_log);
		DSInterface_parse_command();
		count_c = 0;

		//Clear cmd_str
		while (cmd_idx > 0) /* BUG: POTENTIAL BUG for multiple port access */
			cmd_str[--cmd_idx] = '\0';

	} else if ((ch == 0x08 || ch == 0x7F) && cmd_idx > 0) {
		//Backspace character
		if (cmd_idx > 0)
			cmd_str[--cmd_idx] = '\0';
	} else {
		/* BUG: POTENTIAL BUG for multiple port access */
		if (cmd_idx < (int)CONSOLE_STR_BUF_SZ)
			cmd_str[cmd_idx++] = ch;
	}

}

const char cmd_get_bpt_cal[] = "get_cfg bpt cal_result";
void DSInterface_BuildCommand_itself(void)
{
	int len = sizeof(cmd_get_bpt_cal);
	memcpy(&cmd_str[0], cmd_get_bpt_cal, len);
	cmd_str[len] = '\0';
	printLog("%s\r\n", cmd_str);
	DSInterface_parse_command();
}

#define numSampPerBleMtu                 5
void data_report_execute(void)
{

  //const int numSampPerBleMtu = 5; //3
    static uint8_t bleChunkBuffer[BLE_NOTIFY_CHAR_ARR_SIZE + 32];
    static uint8_t bleChunkSampleCnt = 0;


	///char buffer[1792];
	static char buffer[2048] __attribute__((aligned(4)));

	int j, data_len = 0;

	buffer[0] = '\0';

	for (int i = 0; i < num_sensors; i++) {
		//if ((*it)->is_enabled()) {
		data_len = sensor_list[i].data_report_execute(buffer, sizeof(buffer));

		if (/*!this->pause_mode &&*/ (data_len > 0)) {
#if 1
			if ( BLE_Interface_Exists() ) {

				ds_console_interface_exists_ = false;

    			if( data_len == 1245) {

					memcpy( &bleChunkBuffer[bleChunkSampleCnt*data_len], buffer , data_len);
					bleChunkSampleCnt++;

					if( bleChunkSampleCnt == numSampPerBleMtu ){

						BLE_AddtoQueue((uint8_t *)bleChunkBuffer, (int32_t)sizeof(bleChunkBuffer), numSampPerBleMtu*data_len, __LINE__);
						bleChunkSampleCnt = 0;
					}

				}else{
					//printf("data_len = %d %x \r\n " , data_len, *buffer);
					BLE_AddtoQueue((uint8_t *)buffer, (int32_t)sizeof(buffer), data_len, __LINE__);
				}
			}
#endif
			if (ds_console_interface_exists_){
#ifdef ASCII_COMM
				printLog(buffer);
#else
				//printf("data_len = %d %x \r\n " , data_len, *buffer);
				//m_USB->writeBlock((uint8_t*)buffer, data_len);                      //Commented by Jason
                #if 1
                printLog("len=%d, ", data_len);                                  //Added by Jason
                for(j = 0; j < 15/*data_len*/; j++)
                {
                    if(buffer[j] == 0)
                    {
                    	printLog("00 ");
                    }
                    else if(buffer[j] < 16)
                    {
                    	printLog("0");
                    	printLog("%x ", buffer[j]);
                    }
                    else
                    {
                    	printLog("%x ", buffer[j]);
                    }
                }
                printLog("\r\n");                                               //Added by Jason
                #endif
#endif
			}
			data_len = 0;
		}
		//We only support streaming from one device at a time, so break here
		//	break;
		//}
	}
}

void set_fw_version(const char* version)
{
    if (version && *version)
        firmware_version = version;
}

void set_fw_platform(const char* platform)
{
    if (platform && *platform)
        platform_name = platform;
}

