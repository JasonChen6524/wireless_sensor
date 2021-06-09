#include "MaximSensor.h"
#include "Peripherals.h"

#if 0
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
int MaximSensor::readRegister(uint8_t reg, uint8_t *data, int len){
	pr_err("Empty function is called");
	return -1;
}

	/**
	* @brief	Writes data to Maxim Sensor register.
	* @details	Writes data to specific Maxim Sensor register via SPI bus.
	*
	* @param[in]	reg Address of a register to be wrote.
	* @param[in]	data Data to write on register.
	*
	* @returns	0 on success, negative error code on failure.
	*/
int MaximSensor::writeRegister(uint8_t reg,	const uint8_t data){
	pr_err("Empty function is called");
	return -1;
}

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
int MaximSensor::get_part_info(uint8_t *part_id,	uint8_t *rev_id){
	pr_err("Empty function is called");
	return -1;
}

	/**
	* @brief	Enables Maxim Sensor.
	* @details	Enable IRQ, enable LEDs, enable AGC
	*
	* @param[in]	enable Any value to enable, 0 to disable.
	*
	* @returns	0 on success, negative error code on failure.
	*/
int MaximSensor::sensor_enable(int enable){
	pr_err("Empty function is called");
	return -1;
}

	/**
	* @brief	Enables AGC.
	* @details	Enable Maxim Sensor automatic gain controller.
	*	AGC automatically adjusts sampling rates and LED currents to save energy.
	*
	* @param[in]	agc_enable Any value to enable, 0 to disable.
	*
	* @returns	0 on success, negative error code on failure.
	*/
int MaximSensor::agc_enable(int agc_enable){
	pr_err("Empty function is called");
	return -1;
}

	/**
	* @brief	Get sensor part name.
	*
	* @returns	Sensor part name string.
	*/
const char *MaximSensor::get_sensor_part_name(){
	pr_err("Empty function is called");
	return "";
}
#if 0
int dump_registers(addr_val_pair *reg_values) {
	pr_err("Empty function is called");
	return 0;
}
#endif
	/**
	* @brief	Get sensor name.
	*
	* @returns	Sensor name string.
	*/
const char *MaximSensor::get_sensor_name(){
	pr_err("Empty function is called");
	return "";
}

/**
* @brief	Get sensor name.
*
* @returns	Sensor name string.
*/
const char *MaximSensor::get_sensor_algo_ver(){
pr_err("Empty function is called");
return "";
}

#endif
