/*
 * SensorComm.h
 *
 *  Created on: May 28, 2021
 *      Author: jason Chen Local
 */

#ifndef SENSORCOMM_SENSORCOMM_SENSORCOMM_H_
#define SENSORCOMM_SENSORCOMM_SENSORCOMM_H_

#include <stdbool.h>
#include <stdint.h>
#include "app.h"
//#include "DSInterface.h"
#include "MaximSensor.h"

/**
 * @brief	SensorComm is Maxim Sensor Studio GUI command handler base class.
 * @details	SensorComm includes base functions and data structures for to
 *	create new command handler classes. All command handler classes should
 *	implement this class.
 */

void SensorComm_init(uint8_t idx, const char *type, bool visible);

int get_part_info_t(uint8_t *part_id, uint8_t *rev_id);

void SensorComm_create(void);

/*
const char* get_type_t(void);
uint8_t is_visible_t(void);
const char* get_part_name_t(void);
const char* get_algo_ver_t(void);
uint8_t is_enabled_t(void);
void stop_t(void);
uint8_t parse_command_t(const char* cmd);
int data_report_execute_t(char* buf, int size);
void SensorComm_Set_Ble_Status_t(bool status);
*/


typedef struct
{
	get_type_callback get_type;
	get_part_name_callback get_part_name;
	get_algo_ver_callback get_algo_ver;
	get_part_info_callback get_part_info;
	is_visible_callback is_visible;
    //is_enabled_callback is_enabled;
	stop_callback stop;
	parse_command_callback parse_command;
	data_report_execute_callback data_report_execute;
	SensorComm_Set_Ble_Status_callback SensorComm_Set_Ble_Status;

	/* PROTECTED VARIABLES */
	//Mutex comm_mutex;
	//MaximSensor *sensor;

	//int sensor_get_reg(char *ptr_ch, uint8_t *reg_addr, uint8_t *value);
	//int sensor_set_reg(char *ptr_ch);

	uint8_t data_report_mode;
  //volatile uint8_t console_interface_exists;
	//volatile bool m_sensorcomm_ble_interface_exists_;

	int idx;
	const char* sensor_type;
	uint8_t vis;
  //int sample_count;
} SensorComm;

//extern SensorComm sensor_list_t[DS_MAX_NUM_SENSORCOMMS];

#endif /* SENSORCOMM_SENSORCOMM_SENSORCOMM_H_ */
