/*
 * SensorComm.c
 *
 *  Created on: May 28, 2021
 *      Author: jason Chen Local
 */

#include "stddef.h"
#include "stdbool.h"
#include "stdint.h"
#include "SensorComm.h"
#include "MaximSensor.h"
#include "DSInterface.h"
#include "Peripherals.h"

/* PROTECTED VARIABLES */
//Mutex comm_mutex;
MaximSensor *sensor;

//SensorComm sensor_list_t[DS_MAX_NUM_SENSORCOMMS];

//int sensor_get_reg(char *ptr_ch, uint8_t *reg_addr, uint8_t *value);
//int sensor_set_reg(char *ptr_ch);

//volatile uint8_t data_report_mode;
volatile uint8_t console_interface_exists;
volatile bool m_sensorcomm_ble_interface_exists_;

//const char* sensor_type;
//bool vis;
//int sample_count;

void SensorComm_init(uint8_t idx, const char *type, bool visible)
{
}

const char* get_type_t(uint8_t idx)
{
	return NULL;//sensor_list_t[idx].sensor_type;
}

int get_part_info_t(uint8_t *part_id, uint8_t *rev_id)
{
	if (sensor != NULL)
    {
		return sensor->get_part_info(part_id, rev_id);
	} else {
		*part_id = 0xFF;
		*rev_id = 0xFF;
		return -1;
	}
}

uint8_t is_visible_t(uint8_t idx)
{
	return 0;//sensor_list_t[idx].vis;
}

uint8_t is_enabled_t(uint8_t idx)
{
	return 0;//(sensor_list_t[idx].data_report_mode != 0);
}

void SensorComm_Set_Ble_Status_t(bool status, uint8_t idx)
{
	//pr_debug("Setting ble status: %d %10s     ", status, sensor_list_t[idx].sensor_type);                                // Modified by Jason
	//m_sensorcomm_ble_interface_exists_ = status;
}

/*virtual*/const char* get_part_name_t(void)
{
	if (sensor != NULL)
	{
		return sensor->get_sensor_part_name();
	}
	else
	{
		return "unknown";
	}
}

/*virtual*/const char* get_algo_ver_t(void)
{
	if (sensor != NULL)
	{
		return sensor->get_sensor_algo_ver();
	}
	else
	{
		return "unknown";
	}
}

/*virtual*/void stop_t(void)
{
}

/*virtual*/uint8_t parse_command_t(const char* cmd)
{
	return false;
}

/*virtual*/int data_report_execute_t(char* buf, int size)
{
	return 0;
}
