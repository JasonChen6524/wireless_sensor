/*******************************************************************************
* Copyright (C) 2017 Maxim Integrated Products, Inc., All Rights Reserved.
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

#ifndef _MAXIMSENSOR_H_
#define _MAXIMSENSOR_H_

#include <stdint.h>
#include "app.h"

//#include <list>


typedef struct {
    uint8_t addr;
    uint32_t val;
} addr_val_pair;


/**
 * @brief	MaximSensor is Maxim Sensor base class.
 * @details	MaximSensor includes base functions for to create new
 *	sensor classes. All sensor classes should implement this class.
 */
typedef void (*rx_data_callback1)(uint8_t *);
typedef struct {
	int data_size;
	rx_data_callback1 rx_data_parser;
} ss_data_req1;

typedef int  (*readRegister_callback)(uint8_t, uint8_t *, int);
typedef int  (*writeRegister_callback)(uint8_t, uint8_t *, int);
typedef void (*sensor_enable_callback) (uint8_t);
typedef int  (*dump_registers_callback) (addr_val_pair *);
typedef const char* (*get_sensor_part_name_callback)(void);
typedef char* (*get_sensor_algo_ver_callback)(void);

#if 1
typedef struct
{
//public:
	/* PUBLIC FUNCTION DECLARATIONS */
	/**
	* @brief	Reads from register.
	* @details	Reads specific Maxim Sensor register via SPI bus.
	*
	* @param[in]	reg Beginning address of a register to be read.
	* @param[out]	data Buffer space to save result value.
	* @param[in]	len Number of consecutive bytes to be read.
	*
	* @returns	0 on success, negative error code on failure.
	*/
	//int readRegister(uint8_t reg, uint8_t *data, int len);
	readRegister_callback readRegister;

	/**
	* @brief	Writes data to Maxim Sensor register.
	* @details	Writes data to specific Maxim Sensor register via SPI bus.
	*
	* @param[in]	reg Address of a register to be wrote.
	* @param[in]	data Data to write on register.
	*
	* @returns	0 on success, negative error code on failure.
	*/
	//int writeRegister(uint8_t reg,	const uint8_t data);
	writeRegister_callback writeRegister;

	/**
	* @brief	Get Maxim Sensor part and revision info.
	* @details	Reads Maxim Sensor part and revision info from device.
	*
	* @param[in]	reg Beginning address of a register to be read.
	* @param[out]	data Buffer space to save result value.
	* @param[in]	len Number of consecutive bytes to be read.
	*
	* @returns	0 on success, negative error code on failure.
	*/
	//int get_part_info(uint8_t *part_id,	uint8_t *rev_id);
	get_part_info_callback get_part_info;

	/**
	* @brief	Enables Maxim Sensor.
	* @details	Enable IRQ, enable LEDs, enable AGC
	*
	* @param[in]	enable Any value to enable, 0 to disable.
	*
	* @returns	0 on success, negative error code on failure.
	*/
	//int sensor_enable(int enable);
	sensor_enable_callback sensor_enable;


	/**
	* @brief	Enables AGC.
	* @details	Enable Maxim Sensor automatic gain controller.
	*	AGC automatically adjusts sampling rates and LED currents to save energy.
	*
	* @param[in]	agc_enable Any value to enable, 0 to disable.
	*
	* @returns	0 on success, negative error code on failure.
	*/
	//int agc_enable(int agc_enable);
	sensor_enable_callback agc_enable;

	/**
	* @brief	Get sensor part name.
	*
	* @returns	Sensor part name string.
	*/
	//const char *get_sensor_part_name();
	get_sensor_part_name_callback get_sensor_part_name;

	/**
	* @brief	Get sensor algorithm version.
	*
	* @returns	Sensor algorithm version string.
	*/
	//char *get_sensor_algo_ver();
	get_sensor_algo_ver_callback get_sensor_algo_ver;

	/**
	* @brief	Get sensor name.
	*
	* @returns	Sensor name string.
	*/
	//const char *get_sensor_name();
	get_sensor_part_name_callback get_sensor_name;

	/**
	* @brief	Dump Maxim Sensor registers.
	* @details	Print all Maxim Sensor register addresses and containing values.
	*
	* @param[in]    reg_values Pointer to array of 256 addr_val_pairs
	* @returns	0 on success, negative error code on failure.
	*/
	//int dump_registers(addr_val_pair *reg_values)=0;
	dump_registers_callback dump_registers;

} MaximSensor;
#endif

#endif /* _MAXIMSENSOR_H_ */
