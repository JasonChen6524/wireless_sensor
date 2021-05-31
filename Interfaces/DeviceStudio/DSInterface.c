/*
 * DSInterface.c
 *
 *  Created on: May 28, 2021
 *      Author: jason Chen Local
 */
#include <stdint.h>
#include "app.h"
#include "Peripherals.h"
#include "DSInterface.h"
#if 0
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
#endif
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
//SensorComm* sensor_list[DS_MAX_NUM_SENSORCOMMS];

//volatile uint8_t ds_console_interface_exists_;
//volatile uint8_t ds_ble_interface_exists_;

const char* platform_name;
const char* firmware_version;

#define CONSOLE_STR_BUF_SZ                    (2048)

char cmd_str[CONSOLE_STR_BUF_SZ];
int cmd_idx;

//void add_sensor_comm(SensorComm *s)
//{
//	assert_msg(num_sensors < DS_MAX_NUM_SENSORCOMMS, "Too many sensors added to DSInterface. Increase DS_MAX_NUM_SENSORCOMMS.");
//	sensor_list[num_sensors++] = s;
//}

static void DSInterface_parse_command(void)
{

}

void DSInterface_BuildCommand(char ch)
{
	//if (!this->silent_mode) /* BUG: POTENTIAL BUG, what uart port to echo, not only console */
	printLog("%c", ch);

	if (ch == 0x00) {
		pr_err("Ignored char 0x00");
		return;
	}

	if ((ch == '\n') || (ch == '\r')) {
		if (cmd_idx < (int)CONSOLE_STR_BUF_SZ)
           cmd_str[cmd_idx++] = '\0';
		DSInterface_parse_command();

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
