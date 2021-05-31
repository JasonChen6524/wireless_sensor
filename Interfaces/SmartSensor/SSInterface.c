/***************************************************************************
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
****************************************************************************
*/

#include "string.h"
#include "stdio.h"
#include "SSInterface.h"
#include "Peripherals.h"
#include "assert.h"
#include "utils.h"
#include "I2C.h"
#include "app.h"
//#include "ble/BLE.h"

//#define DEBUG_TIMING_PROFILE

char fw_version[128];
char algo_version[128];
const char* plat_name;

static bool in_bootldr;
static bool sc_en;
static int data_type;

static volatile bool m_irq_received_;
static volatile bool mfio_int_happened;

static int sensor_enabled_mode[SS_MAX_SUPPORTED_SENSOR_NUM];
static ss_data_req* sensor_data_reqs[SS_MAX_SUPPORTED_SENSOR_NUM];

static void fifo_sample_size(int data_type, int* sample_size);
static int algo_enabled_mode[SS_MAX_SUPPORTED_ALGO_NUM];
static ss_data_req* algo_data_reqs[SS_MAX_SUPPORTED_ALGO_NUM];

#ifdef DEBUG_TIMING_PROFILE
	Timer debugTimer;
	static int prevTime_ms;
#endif

#if 0
SSInterface::SSInterface(I2C &i2cBus, PinName ss_mfio, PinName ss_reset)
	:m_i2cBus(&i2cBus), m_spiBus(NULL),
	mfio_pin(ss_mfio), reset_pin(ss_reset), irq_pin(ss_mfio)/*,
	irq_evt(1000000, "irq")*/
{
	reset_pin.input();
#ifndef BPT_POLL_MODE
	irq_pin.fall(callback(this, &SSInterface::irq_handler));
#endif
	reset_to_main_app();
	get_data_type(&data_type, &sc_en);
#ifdef DEBUG_TIMING_PROFILE
	debugTimer.start();
#endif

}

SSInterface::SSInterface(SPI &spiBus, PinName ss_mfio, PinName ss_reset)
	:m_i2cBus(NULL), m_spiBus(&spiBus),
	mfio_pin(ss_mfio), reset_pin(ss_reset), irq_pin(ss_mfio)/*,
	irq_evt(1000000, "irq")*/
{
	reset_pin.input();
#ifndef BPT_POLL_MODE
	irq_pin.fall(callback(this, &SSInterface::irq_handler));
#endif
	reset_to_main_app();
	get_data_type(&data_type, &sc_en);
}

SSInterface::~SSInterface()
{
}
#endif

SS_STATUS reset_to_main_app(void)
{
	ss_disable_irq();
#if defined(BOOTLOADER_USES_MFIO)
	reset_pin.output();
	cfg_mfio(PIN_OUTPUT);
	reset_pin.write(0);
	wait_ms(SS_RESET_TIME);
	mfio_pin.write(1);
	reset_pin.write(1);
	wait_ms(SS_STARTUP_TO_MAIN_APP_TIME);
	cfg_mfio(PIN_INPUT);
	reset_pin.input();
	enable_irq();
	// Verify we exited bootloader mode
	if (in_bootldr_mode() == 0)
		return SS_SUCCESS;
	else
		return SS_ERR_UNKNOWN;
#else
	SS_STATUS status = ss_exit_from_bootloader();
	ss_enable_irq();
	return status;
#endif
}

SS_STATUS reset_to_bootloader(void)
{
	ss_disable_irq();
#if defined(BOOTLOADER_USES_MFIO)
	reset_pin.output();
	cfg_mfio(PIN_OUTPUT);
	reset_pin.write(0);
	wait_ms(SS_RESET_TIME);
	mfio_pin.write(0);
	reset_pin.write(1);
	wait_ms(SS_STARTUP_TO_BTLDR_TIME);
	cfg_mfio(PIN_INPUT);
	reset_pin.input();
	ss_enable_irq();
	stay_in_bootloader();

	// Verify we entered bootloader mode
	if (in_bootldr_mode() < 0)
		return SS_ERR_UNKNOWN;
	return SS_SUCCESS;
#else
	ss_stay_in_bootloader();
	ss_enable_irq();
	return SS_SUCCESS;
#endif
}

SS_STATUS ss_exit_from_bootloader()
{
	uint8_t cmd_bytes[] = { SS_FAM_W_MODE, SS_CMDIDX_MODE };
	uint8_t data[] = { 0x00 };

	SS_STATUS status = write_cmd2(
			&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
			&data[0], ARRAY_SIZE(data), SS_DEFAULT_CMD_SLEEP_MS);

	in_bootldr = (status == SS_SUCCESS) ? true : false;
	return status;
}

SS_STATUS ss_stay_in_bootloader(void)
{
	uint8_t cmd_bytes[] = { SS_FAM_W_MODE, SS_CMDIDX_MODE };
	uint8_t data[] = { SS_MASK_MODE_BOOTLDR };

	SS_STATUS status = write_cmd2(
			&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
			&data[0], ARRAY_SIZE(data), SS_DEFAULT_CMD_SLEEP_MS);

	in_bootldr = (status == SS_SUCCESS) ? true : false;
	return status;
}

int ss_in_bootldr_mode(void)
{
	uint8_t cmd_bytes[] = { SS_FAM_R_MODE, SS_CMDIDX_MODE };
	uint8_t rxbuf[2] = { 0 };

	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
			0, 0,
			&rxbuf[0], ARRAY_SIZE(rxbuf), SS_DEFAULT_CMD_SLEEP_MS);
	if (status != SS_SUCCESS)
		return -1;

	return (rxbuf[1] & SS_MASK_MODE_BOOTLDR);
}

SS_STATUS reset(void)
{
	int bootldr = ss_in_bootldr_mode();
	if (bootldr > 0)
		return reset_to_bootloader();
	else if (bootldr == 0)
		return reset_to_main_app();
	else
		return SS_ERR_UNKNOWN;
}

SS_STATUS self_test(int idx, uint8_t *result, int sleep_ms){
    uint8_t cmd_bytes[] = { SS_FAM_R_SELFTEST, (uint8_t)idx };
    uint8_t rxbuf[2];
    SS_STATUS ret;

	result[0] = 0xFF;
	ret = read_cmd(cmd_bytes, 2, (uint8_t *)0, 0, rxbuf, ARRAY_SIZE(rxbuf), sleep_ms);
	result[0] = rxbuf[1];
	return ret;
}

void cfg_mfio(PinDirection dir)
{
	if (dir == PIN_INPUT)
	{
		//??mfio_pin.input();
		//mfio_pin.mode(PullUp);
	}
	else
	{
		ss_disable_irq();
		//??mfio_pin.output();
	}
}

void ss_enable_irq(void)
{
#ifndef BPT_POLL_MODE
	irq_pin.enable_irq();
#endif
}
void ss_disable_irq(void)
{
#ifndef BPT_POLL_MODE
	irq_pin.disable_irq();
#endif
}

void ss_mfio_selftest(void){
	ss_disable_irq();
#ifndef BPT_POLL_MODE
	irq_pin.fall(callback(this, &SSInterface::irq_handler_selftest));
#endif
	ss_enable_irq();
}

const char* get_ss_fw_version(void)
{
    uint8_t cmd_bytes[2];
    uint8_t rxbuf[4];

	int bootldr = ss_in_bootldr_mode();

	if (bootldr > 0) {
		cmd_bytes[0] = SS_FAM_R_BOOTLOADER;
		cmd_bytes[1] = SS_CMDIDX_BOOTFWVERSION;
	} else if (bootldr == 0) {
		cmd_bytes[0] = SS_FAM_R_IDENTITY;
		cmd_bytes[1] = SS_CMDIDX_FWVERSION;
	} else {
		return plat_name;
	}

    SS_STATUS status = read_cmd(
             &cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
             0, 0,
             &rxbuf[0], ARRAY_SIZE(rxbuf), SS_DEFAULT_CMD_SLEEP_MS);

    if (status == SS_SUCCESS) {
        snprintf(fw_version, sizeof(fw_version),
            "%d.%d.%d", rxbuf[1], rxbuf[2], rxbuf[3]);
		pr_info("fw_version:%s\r\n", fw_version);
    }

    return &fw_version[0];
}

const char* get_ss_algo_version(void)
{
    uint8_t cmd_bytes[3];
    uint8_t rxbuf[4];

	int bootldr = ss_in_bootldr_mode();

	if (bootldr > 0) {
		cmd_bytes[0] = SS_FAM_R_BOOTLOADER;
		cmd_bytes[1] = SS_CMDIDX_BOOTFWVERSION;
		cmd_bytes[2] = 0;
	} else if (bootldr == 0) {
		cmd_bytes[0] = SS_FAM_R_IDENTITY;
		cmd_bytes[1] = SS_CMDIDX_ALGOVER;
		cmd_bytes[2] = SS_CMDIDX_AVAILSENSORS;
	} else {
		return plat_name;
	}

    SS_STATUS status = read_cmd(
             &cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
             0, 0,
             &rxbuf[0], ARRAY_SIZE(rxbuf), SS_DEFAULT_CMD_SLEEP_MS);

    if (status == SS_SUCCESS) {
        snprintf(algo_version, sizeof(algo_version),
            "%d.%d.%d", rxbuf[1], rxbuf[2], rxbuf[3]);
		pr_info("algo_version:%s\r\n", fw_version);
    }

    return &algo_version[0];
}
const char* get_ss_platform_name(void)
{
    uint8_t cmd_bytes[] = { SS_FAM_R_IDENTITY, SS_CMDIDX_PLATTYPE };
    uint8_t rxbuf[2];

    SS_STATUS status = read_cmd(
            &cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
            0, 0,
            &rxbuf[0], ARRAY_SIZE(rxbuf), SS_DEFAULT_CMD_SLEEP_MS);

    if (status == SS_SUCCESS) {
        if (rxbuf[1] == SS_PLAT_MAX3263X) {
			if (ss_in_bootldr_mode() > 0) {
				plat_name = SS_BOOTLOADER_PLATFORM_MAX3263X;
			} else {
	            plat_name = SS_PLATFORM_MAX3263X;
			}
        } else if (rxbuf[1] == SS_PLAT_MAX32660) {
			if (ss_in_bootldr_mode() > 0) {
				plat_name = SS_BOOTLOADER_PLATFORM_MAX32660;
			} else {
            	plat_name = SS_PLATFORM_MAX32660;
			}
        }
    }

    return plat_name;
}

static SS_STATUS write_cmd_small(uint8_t *cmd_bytes, int cmd_bytes_len,
                       uint8_t *data, int data_len,
                       int sleep_ms)
{
    uint8_t write_buf[SS_SMALL_BUF_SIZE];
    memcpy(write_buf, cmd_bytes, cmd_bytes_len);
    memcpy(write_buf + cmd_bytes_len, data, data_len);

	SS_STATUS status = write_cmd(write_buf, cmd_bytes_len + data_len, sleep_ms);
	return status;
}

static SS_STATUS write_cmd_medium(uint8_t *cmd_bytes, int cmd_bytes_len,
                       uint8_t *data, int data_len,
                       int sleep_ms)
{
    uint8_t write_buf[SS_MED_BUF_SIZE];
    memcpy(write_buf, cmd_bytes, cmd_bytes_len);
    memcpy(write_buf + cmd_bytes_len, data, data_len);

	SS_STATUS status = write_cmd(write_buf, cmd_bytes_len + data_len, sleep_ms);
	return status;
}

static SS_STATUS write_cmd_large(uint8_t *cmd_bytes, int cmd_bytes_len,
                       uint8_t *data, int data_len,
                       int sleep_ms)
{
    uint8_t write_buf[SS_LARGE_BUF_SIZE];
    memcpy(write_buf, cmd_bytes, cmd_bytes_len);
    memcpy(write_buf + cmd_bytes_len, data, data_len);

	SS_STATUS status = write_cmd(write_buf, cmd_bytes_len + data_len, sleep_ms);
	return status;
}

SS_STATUS write_cmd2(uint8_t *cmd_bytes, int cmd_bytes_len, uint8_t *data, int data_len, int sleep_ms)
{
    int total_len = data_len + cmd_bytes_len;

    if (total_len <= SS_SMALL_BUF_SIZE) {
        return write_cmd_small(cmd_bytes, cmd_bytes_len, data, data_len, sleep_ms);
    } else if (total_len <= SS_MED_BUF_SIZE) {
        return write_cmd_medium(cmd_bytes, cmd_bytes_len, data, data_len, sleep_ms);
    } else if (total_len <= SS_LARGE_BUF_SIZE) {
        return write_cmd_large(cmd_bytes, cmd_bytes_len, data, data_len, sleep_ms);
    } else {
        assert_msg(true, "Tried to send I2C tx larger than maximum allowed size\n");
        return SS_ERR_DATA_FORMAT; 
    }
}

SS_STATUS write_cmd(uint8_t *tx_buf, int tx_len, int sleep_ms)
{
	pr_info("write_cmd: ");
	for (int i = 0; i < tx_len; i++) {
		pr_info("0x%02X ", tx_buf[i]);
	}
	pr_info("\r\n");

    int ret = m_i2cBus_write(SS_I2C_8BIT_SLAVE_ADDR, tx_buf, tx_len, false);

	int retries = 4;
	while (ret != 0 && retries-- > 0) {
		pr_err("i2c wr retry\r\n");
		wait_ms(1);
    	ret = m_i2cBus_write(SS_I2C_8BIT_SLAVE_ADDR, tx_buf, tx_len, false);
	}

    if (ret != 0) {
    	pr_err("m_i2cBus->write returned %d\r\n", ret);
        return SS_ERR_UNAVAILABLE;
    }

    wait_ms(sleep_ms);

    uint8_t status_byte;
    ret = m_i2cBus_read(SS_I2C_8BIT_SLAVE_ADDR, &status_byte, 1);
	bool try_again = (status_byte == SS_ERR_TRY_AGAIN);
	while ((ret != 0 || try_again) 
			&& retries-- > 0) {
		pr_info("i2c rd retry\r\n");
		wait_ms(sleep_ms);
    	ret = m_i2cBus_read(SS_I2C_8BIT_SLAVE_ADDR, &status_byte, 1);
		try_again = (status_byte == SS_ERR_TRY_AGAIN);
	}

    if (ret != 0 || try_again) {
    	pr_err("m_i2cBus->read returned %d, ss status_byte %d\r\n", ret, status_byte);
        return SS_ERR_UNAVAILABLE;
    }

	pr_info("status_byte: %d\r\n", status_byte);

	return (SS_STATUS)status_byte;
}

SS_STATUS read_cmd(uint8_t *cmd_bytes, int cmd_bytes_len,
	uint8_t *data, int data_len,
	uint8_t *rxbuf, int rxbuf_sz,
    int sleep_ms)
{
	//pr_info("read_cmd: ");                                // by Jason   Begin 
	//for (int i = 0; i < cmd_bytes_len; i++) {
	//	pr_info("0x%02X ", cmd_bytes[i]);
	//}
	//pr_info("\r\n");                                      // by Jason   End


	int retries = 4;

    int ret = m_i2cBus_write(SS_I2C_8BIT_SLAVE_ADDR, cmd_bytes, cmd_bytes_len, (data_len != 0));
#ifdef SHOW_I2C_DEBUG_MESSAGES
    printf("ret1 : %d\rt\n",ret);
#endif
    if (data_len != 0) {
        ret |= m_i2cBus_write(SS_I2C_8BIT_SLAVE_ADDR, data, data_len, false);
#ifdef SHOW_I2C_DEBUG_MESSAGES
        printf("ret2 : %d\rt\n",ret);
#endif
    }

	while (ret != 0 && retries-- > 0) {

		pr_err("i2c wr retry\r\n");
		wait_ms(1);
    	ret = m_i2cBus_write(SS_I2C_8BIT_SLAVE_ADDR, cmd_bytes, cmd_bytes_len, (data_len != 0));
#ifdef SHOW_I2C_DEBUG_MESSAGES
    	printf("ret3 : %d\rt\n",ret);
#endif
	    if (data_len != 0) {
	        ret |= m_i2cBus_write(SS_I2C_8BIT_SLAVE_ADDR, data, data_len, false);
#ifdef SHOW_I2C_DEBUG_MESSAGES
	        printf("ret4 : %d\rt\n",ret);
#endif
	    }
	}

    if (ret != 0) {
    	pr_err("m_i2cBus->write returned %d\r\n", ret);
        return SS_ERR_UNAVAILABLE;
    }

    wait_ms(sleep_ms);

    ret = m_i2cBus_read(SS_I2C_8BIT_SLAVE_ADDR, rxbuf, rxbuf_sz);
	bool try_again = (rxbuf[0] == SS_ERR_TRY_AGAIN);
	while ((ret != 0 || try_again) && retries-- > 0) {
		pr_info("i2c rd retry\r\n");
		wait_ms(sleep_ms);
    	ret = m_i2cBus_read(SS_I2C_8BIT_SLAVE_ADDR, rxbuf, rxbuf_sz);
		try_again = (rxbuf[0] == SS_ERR_TRY_AGAIN);
	}
    if (ret != 0 || try_again) {
    	pr_err("m_i2cBus->read returned %d, ss status_byte %d\r\n", ret, rxbuf[0]);
        return SS_ERR_UNAVAILABLE;
    }

	pr_info("status_byte: %d\r\n", rxbuf[0]);
	pr_info("data:");                                            
	for (int i = 1; i < rxbuf_sz; i++) {                          
		pr_info("0x%02X ", rxbuf[i]);
	}
	pr_info("\r\n");

    return (SS_STATUS)rxbuf[0];
}

SS_STATUS get_reg(int idx, uint8_t addr, uint32_t *val)
{
	assert_msg((idx <= SS_MAX_SUPPORTED_SENSOR_NUM), "idx must be < SS_MAX_SUPPORTED_SENSOR_NUM, or update code to handle variable length idx values");

	uint8_t cmd_bytes[] = { SS_FAM_R_REGATTRIBS, (uint8_t)idx };
	uint8_t rx_reg_attribs[3] = {0};

	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
								0, 0,
								&rx_reg_attribs[0], ARRAY_SIZE(rx_reg_attribs), SS_DEFAULT_CMD_SLEEP_MS);

	if (status != SS_SUCCESS)
		return status;

	int reg_width = rx_reg_attribs[1];

	uint8_t cmd_bytes2[] = { SS_FAM_R_READREG, (uint8_t)idx, addr };
	uint8_t rxbuf[5] = {0};

	status = read_cmd(&cmd_bytes2[0], ARRAY_SIZE(cmd_bytes2),
						0, 0,
						&rxbuf[0], reg_width + 1, SS_DEFAULT_CMD_SLEEP_MS);

	if (status == SS_SUCCESS) {
		*val = 0;
		for (int i = 0; i < reg_width; i++) {
			*val = (*val << 8) | rxbuf[i + 1];
		}
	}

	return status;
}

SS_STATUS set_reg(int idx, uint8_t addr, uint32_t val, int byte_size)
{
	assert_msg((idx <= SS_MAX_SUPPORTED_SENSOR_NUM), "idx must be < SS_MAX_SUPPORTED_SENSOR_NUM, or update code to handle variable length idx values");

	uint8_t cmd_bytes[] = { SS_FAM_W_WRITEREG, (uint8_t)idx, addr };
	uint8_t data_bytes[4];
	for (int i = 0; i < byte_size; i++) {
		data_bytes[i] = (val >> (8 * (byte_size - 1)) & 0xFF);
	}

	SS_STATUS status = write_cmd2(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
								&data_bytes[0], byte_size, SS_ENABLE_SENSOR_SLEEP_MS);

	return status;
}

SS_STATUS dump_reg(int idx, addr_val_pair* reg_vals, int reg_vals_sz, int* num_regs)
{
	assert_msg((idx <= SS_MAX_SUPPORTED_SENSOR_NUM), "idx must be < SS_MAX_SUPPORTED_SENSOR_NUM, or update code to handle variable length idx values");

	uint8_t cmd_bytes[] = { SS_FAM_R_REGATTRIBS, (uint8_t)idx };
	uint8_t rx_reg_attribs[3] = {0};

	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
								0, 0,
								&rx_reg_attribs[0], ARRAY_SIZE(rx_reg_attribs), SS_DEFAULT_CMD_SLEEP_MS);

	if (status != SS_SUCCESS)
		return status;

	int reg_width = rx_reg_attribs[1];
	*num_regs = rx_reg_attribs[2];
	assert_msg((*num_regs <= reg_vals_sz), "Need to increase reg_vals array to hold all dump_reg data");
	assert_msg(((size_t)reg_width <= sizeof(uint32_t)), "IC returned register values greater than 4 bytes in width");

	int dump_reg_sz = (*num_regs) * (reg_width + 1) + 1; //+1 to reg_width for address, +1 for status byte

	uint8_t rxbuf[512];
	assert_msg(((size_t)dump_reg_sz <= sizeof(rxbuf)), "Need to increase buffer size to receive dump_reg data");

	cmd_bytes[0] = SS_FAM_R_DUMPREG;
	status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
								0, 0,
								&rxbuf[0], dump_reg_sz, SS_DUMP_REG_SLEEP_MS);

	if (status != SS_SUCCESS)
		return status;

	//rxbuf format is [status][addr0](reg_width x [val0])[addr1](reg_width x [val1])...
	for (int reg = 0; reg < *num_regs; reg++) {
		reg_vals[reg].addr = rxbuf[(reg * (reg_width + 1)) + 1];
		uint32_t *val = &(reg_vals[reg].val);
		*val = 0;
		for (int byte = 0; byte < reg_width; byte++) {
			*val = (*val << 8) | rxbuf[(reg * (reg_width + 1)) + byte + 2];
		}
	}

	return SS_SUCCESS;
}

SS_STATUS enable_sensor(int idx, int mode, ss_data_req *data_req)
{
	assert_msg((idx <= SS_MAX_SUPPORTED_SENSOR_NUM), "idx must be < SS_MAX_SUPPORTED_SENSOR_NUM, or update code to handle variable length idx values");
	assert_msg((mode <= SS_MAX_SUPPORTED_MODE_NUM), "mode must be < SS_MAX_SUPPORTED_MODE_NUM, or update code to handle variable length mode values");
	assert_msg((mode != 0), "Tried to enable sensor to mode 0, but mode 0 is disable");


	uint8_t cmd_bytes[] = { SS_FAM_W_SENSORMODE, (uint8_t)idx, (uint8_t)mode };

	SS_STATUS status = write_cmd2(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes), 0, 0, 5 * SS_ENABLE_SENSOR_SLEEP_MS);

	if (status == SS_SUCCESS) {
		sensor_enabled_mode[idx] = mode;
		sensor_data_reqs[idx] = data_req;
	}
	return status;
}

SS_STATUS disable_sensor(int idx)
{
	assert_msg((idx <= SS_MAX_SUPPORTED_SENSOR_NUM), "idx must be < SS_MAX_SUPPORTED_SENSOR_NUM, or update code to handle variable length idx values");
	uint8_t cmd_bytes[] = { SS_FAM_W_SENSORMODE, (uint8_t)idx, 0 };

	SS_STATUS status = write_cmd2(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes), 0, 0, SS_ENABLE_SENSOR_SLEEP_MS);

	if (status == SS_SUCCESS) {
		sensor_enabled_mode[idx] = 0;
		sensor_data_reqs[idx] = 0;
	}

	return status;
}

SS_STATUS enable_algo(int idx, int mode, ss_data_req *data_req)
{
	assert_msg((idx <= SS_MAX_SUPPORTED_ALGO_NUM), "idx must be < SS_MAX_SUPPORTED_ALGO_NUM, or update code to handle variable length idx values");
	assert_msg((mode <= SS_MAX_SUPPORTED_MODE_NUM), "mode must be < SS_MAX_SUPPORTED_MODE_NUM, or update code to handle variable length mode values");
	assert_msg((mode != 0), "Tried to enable algo to mode 0, but mode 0 is disable");

	uint8_t cmd_bytes[] = { SS_FAM_W_ALGOMODE, (uint8_t)idx, (uint8_t)mode };

	SS_STATUS status = write_cmd2(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes), 0, 0, 25 * SS_ENABLE_SENSOR_SLEEP_MS);

	if (status == SS_SUCCESS) {
		algo_enabled_mode[idx] = mode;
		algo_data_reqs[idx] = data_req;
	}

	return status;
}

SS_STATUS disable_algo(int idx)
{
	assert_msg((idx <= SS_MAX_SUPPORTED_ALGO_NUM), "idx must be < SS_MAX_SUPPORTED_ALGO_NUM, or update code to handle variable length idx values");
	uint8_t cmd_bytes[] = { SS_FAM_W_ALGOMODE, (uint8_t)idx, 0 };

	SS_STATUS status = write_cmd2(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes), 0, 0, SS_ENABLE_SENSOR_SLEEP_MS);

	if (status == SS_SUCCESS) {
		algo_enabled_mode[idx] = 0;
		algo_data_reqs[idx] = 0;
	}

	return status;
}

SS_STATUS set_algo_cfg(int algo_idx, int cfg_idx, uint8_t *cfg, int cfg_sz)
{
	assert_msg((algo_idx <= SS_MAX_SUPPORTED_ALGO_NUM), "idx must be < SS_MAX_SUPPORTED_ALGO_NUM, or update code to handle variable length idx values");
	assert_msg((cfg_idx <= SS_MAX_SUPPORTED_ALGO_CFG_NUM), "idx must be < SS_MAX_SUPPORTED_ALGO_CFG_NUM, or update code to handle variable length idx values");

	uint8_t cmd_bytes[] = { SS_FAM_W_ALGOCONFIG, (uint8_t)algo_idx, (uint8_t)cfg_idx };
	SS_STATUS status = write_cmd2(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes), cfg, cfg_sz, 30);

	return status;
}

SS_STATUS get_algo_cfg(int algo_idx, int cfg_idx, uint8_t *cfg, int cfg_sz)
{
	assert_msg((algo_idx <= SS_MAX_SUPPORTED_ALGO_NUM), "idx must be < SS_MAX_SUPPORTED_ALGO_NUM, or update code to handle variable length idx values");
	assert_msg((cfg_idx <= SS_MAX_SUPPORTED_ALGO_CFG_NUM), "idx must be < SS_MAX_SUPPORTED_ALGO_CFG_NUM, or update code to handle variable length idx values");

	uint8_t cmd_bytes[] = { SS_FAM_R_ALGOCONFIG, (uint8_t)algo_idx, (uint8_t)cfg_idx };
	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes), 0, 0, cfg, cfg_sz, SS_DEFAULT_CMD_SLEEP_MS);

	return status;
}

SS_STATUS set_data_type(int data_type1, bool sc_en1)
{
	assert_msg((data_type >= 0) && (data_type <= 3), "Invalid value for data_type");
	uint8_t cmd_bytes[] = { SS_FAM_W_COMMCHAN, SS_CMDIDX_OUTPUTMODE };
	uint8_t data_bytes[] = { (uint8_t)((sc_en ? SS_MASK_OUTPUTMODE_SC_EN : 0) |
							((data_type << SS_SHIFT_OUTPUTMODE_DATATYPE) & SS_MASK_OUTPUTMODE_DATATYPE)) };

	SS_STATUS status = write_cmd2(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes), &data_bytes[0], ARRAY_SIZE(data_bytes), SS_DEFAULT_CMD_SLEEP_MS);
	data_type = data_type1;
	sc_en     = sc_en1;

	return status;
}


SS_STATUS get_data_type(int *data_type1, bool *sc_en1)
{
	uint8_t cmd_bytes[] = { SS_FAM_R_COMMCHAN, SS_CMDIDX_OUTPUTMODE };
	uint8_t rxbuf[2] = {0};

	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes), 0, 0, &rxbuf[0], ARRAY_SIZE(rxbuf), SS_DEFAULT_CMD_SLEEP_MS);
	if (status == SS_SUCCESS) {
		*data_type1 =
			(rxbuf[1] & SS_MASK_OUTPUTMODE_DATATYPE) >> SS_SHIFT_OUTPUTMODE_DATATYPE;
		*sc_en1 =
			(bool)((rxbuf[1] & SS_MASK_OUTPUTMODE_SC_EN) >> SS_SHIFT_OUTPUTMODE_SC_EN);
	}

	return status;
}

SS_STATUS set_fifo_thresh(int thresh)
{
	assert_msg((thresh > 0 && thresh <= 255), "Invalid value for fifo a full threshold");
	uint8_t cmd_bytes[] = { SS_FAM_W_COMMCHAN, SS_CMDIDX_FIFOAFULL };
	uint8_t data_bytes[] = { (uint8_t)thresh };

	SS_STATUS status = write_cmd2(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
								&data_bytes[0], ARRAY_SIZE(data_bytes), SS_DEFAULT_CMD_SLEEP_MS);
	return status;
}

SS_STATUS get_fifo_thresh(int *thresh)
{
	uint8_t cmd_bytes[] = { SS_FAM_R_COMMCHAN, SS_CMDIDX_FIFOAFULL };
	uint8_t rxbuf[2] = {0};

	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
								0, 0,
								&rxbuf[0], ARRAY_SIZE(rxbuf), SS_DEFAULT_CMD_SLEEP_MS);

	if (status == SS_SUCCESS) {
		*thresh = rxbuf[1];
	}

	return status;
}

SS_STATUS ss_comm_check(void)
{
	uint8_t cmd_bytes[] = { SS_FAM_R_IDENTITY, SS_CMDIDX_PLATTYPE };
	uint8_t rxbuf[2];

	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
								0, 0,
								&rxbuf[0], ARRAY_SIZE(rxbuf), SS_DEFAULT_CMD_SLEEP_MS);

	int tries = 4;
	while (status == SS_ERR_TRY_AGAIN && tries--) {
		wait_ms(1000);
		status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
							0, 0,
							&rxbuf[0], ARRAY_SIZE(rxbuf), SS_DEFAULT_CMD_SLEEP_MS);
	}

	return status;
}

static void fifo_sample_size(int data_type, int *sample_size)
{
	*sample_size = 0;

	if (data_type == SS_DATATYPE_RAW || data_type == SS_DATATYPE_BOTH) {
		for (int i = 0; i < SS_MAX_SUPPORTED_SENSOR_NUM; i++) {
			if (sensor_enabled_mode[i])
			{
				assert_msg(sensor_data_reqs[i], "no ss_data_req found for enabled sensor");
				*sample_size += sensor_data_reqs[i]->data_size;
			}
		}
	}

	if (data_type == SS_DATATYPE_ALGO || data_type == SS_DATATYPE_BOTH) {
		for (int i = 0; i < SS_MAX_SUPPORTED_ALGO_NUM; i++) {
			if(algo_enabled_mode[i])
			{
				assert_msg(algo_data_reqs[i], "no ss_data_req found for enabled algo");
				*sample_size += algo_data_reqs[i]->data_size;
			}
		}
	}
}

SS_STATUS num_avail_samples(int *num_samples)
{
	uint8_t cmd_bytes[] = { SS_FAM_R_OUTPUTFIFO, SS_CMDIDX_OUT_NUMSAMPLES };
	uint8_t rxbuf[2] = {0};

	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
								0, 0,
								&rxbuf[0], ARRAY_SIZE(rxbuf), 1);

	if (status == SS_SUCCESS) {
		*num_samples = rxbuf[1];
	}

	return status;
}

SS_STATUS get_log_len(int *log_len)
{
	uint8_t cmd_bytes[] = { SS_FAM_R_LOG, SS_CMDIDX_R_LOG_LEN };
	uint8_t rxbuf[2] = {0};

	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
								0, 0,
								&rxbuf[0], ARRAY_SIZE(rxbuf), 1);

	if (status == SS_SUCCESS) {
		*log_len = (rxbuf[1] << 8) | rxbuf[0];
	}

	return status;
}

SS_STATUS read_fifo_data(
	int num_samples, int sample_size,
	uint8_t* databuf, int databuf_sz)
{
	int bytes_to_read = num_samples * sample_size + 1; //+1 for status byte
	assert_msg((bytes_to_read <= databuf_sz), "databuf too small");

	uint8_t cmd_bytes[] = { SS_FAM_R_OUTPUTFIFO, SS_CMDIDX_READFIFO };

	pr_info("[reading %d bytes (%d samples)\r\n", bytes_to_read, num_samples);

	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
								0, 0,
								databuf, bytes_to_read, 10);

	return status;
}

SS_STATUS read_ss_log(int num_bytes, uint8_t *log_buf, int log_buf_sz)
{
	int bytes_to_read = num_bytes + 1; //+1 for status byte
	assert_msg((bytes_to_read <= log_buf_sz), "log_buf too small");

	uint8_t cmd_bytes[] = { SS_FAM_R_LOG, SS_CMDIDX_R_LOG_DATA };

	pr_info("[reading %d bytes (%d samples)\r\n", bytes_to_read, bytes_to_read);

	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
								0, 0,
								log_buf, bytes_to_read, 5);

	return status;
}



static uint8_t databuf[1024];
void ss_execute_once(void){

	if(m_irq_received_ == false)
		return;

#ifdef DEBUG_TIMING_PROFILE
		int currTime = debugTimer.read_ms();
        printf("ssevent_t= %d |   ", currTime - prevTime_ms);
        prevTime_ms = currTime;
#endif

	uint8_t sample_count;
	m_irq_received_ = false;
	uint8_t cmd_bytes[] = { SS_FAM_R_STATUS, SS_CMDIDX_STATUS };
	uint8_t rxbuf[2] = {0};

	//irq_evt.start();

	ss_disable_irq();

	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
								0, 0,
								&rxbuf[0], ARRAY_SIZE(rxbuf), SS_DEFAULT_CMD_SLEEP_MS);
	if (status != SS_SUCCESS) {
		pr_err("Couldn't read status byte of SmartSensor!");
		ss_enable_irq();
		pr_err("%s:%d\n", __func__, __LINE__);
		//irq_evt.stop();
#ifdef DEBUG_TIMING_PROFILE
        printf("\r\n");
#endif
		return;
	}

	//BLE::Instance().waitForEvent();
	//printf("status= %d \r\n" , rxbuf[1]);

	if (rxbuf[1] & SS_MASK_STATUS_ERR) {
		pr_err("SmartSensor status error: %d", rxbuf[1] & SS_MASK_STATUS_ERR);
	}
	if (rxbuf[1] & SS_MASK_STATUS_FIFO_OUT_OVR) {
		pr_err("SmartSensor Output FIFO overflow!");
	}
	if (rxbuf[1] & SS_MASK_STATUS_FIFO_IN_OVR) {
		pr_err("SmartSensor Input FIFO overflow!");
	}

	if (rxbuf[1] & SS_MASK_STATUS_LOG_OVR) {
		pr_err("SmartSensor log overflow!");
	}

	if (rxbuf[1] & SS_MASK_STATUS_LOG_RDY) {

		pr_err("SmartSensor Log ready");
		int log_len;
		status = get_log_len(&log_len);
		if (status != SS_SUCCESS)
		{
			pr_err("Couldn't read log lenght");
			ss_enable_irq();
			//irq_evt.stop();
#ifdef DEBUG_TIMING_PROFILE
            printf("\r\n");
#endif            
			return;
		}

		pr_err("log_len: %d\n", log_len);
		assert_msg((log_len <= sizeof(databuf)), "log size in SS longer than buffer.\n");
		status = read_ss_log(log_len, &databuf[0], sizeof(databuf));
		if (status != SS_SUCCESS)
		{
			pr_err("Couldn't read from SmartSensor Log");
			ss_enable_irq();
			//irq_evt.stop();
#ifdef DEBUG_TIMING_PROFILE
            printf("\r\n");
#endif            
			return;
		}

		databuf[log_len] = 0;
		printLog("\r\n--->%s", (char *)databuf);
	}

	if (rxbuf[1] & SS_MASK_STATUS_DATA_RDY) {

		int num_samples = 1;
		status = num_avail_samples(&num_samples);
		if (status != SS_SUCCESS)
		{
			pr_err("Couldn't read number of available samples in SmartSensor Output FIFO");
			ss_enable_irq();
			//irq_evt.stop();
#ifdef DEBUG_TIMING_PROFILE
            printf("\r\n");
#endif            
			return;
		}

		//BLE::Instance().waitForEvent();

#ifdef DEBUG_TIMING_PROFILE
		printf("Num of sample = %d \r\n", num_samples);
#endif
		//printf("nsamp = %d \r\n", num_samples);

		int sample_size;
		fifo_sample_size(data_type, &sample_size);

		int bytes_to_read = num_samples * sample_size + 1; //+1 for status byte
		if ((uint32_t)bytes_to_read > sizeof(databuf)) {
			//Reduce number of samples to read to fit in buffer
			num_samples = (sizeof(databuf) - 1) / sample_size;
		}

		//wait_ms(5);

		//printf("nbytes to read  = %d \r\n", bytes_to_read);

		status = read_fifo_data(num_samples, sample_size, &databuf[0], sizeof(databuf));
		if (status != SS_SUCCESS)
		{
			printf("Couldn't read from SmartSensor Output FIFO");
			ss_enable_irq();
			//irq_evt.stop();
			return;
		}

		//BLE::Instance().waitForEvent();

		//Skip status byte
		uint8_t *data_ptr = &databuf[1];

		int i = 0;
		for (i = 0; i < num_samples; i++) {
			if (sc_en) {
				sample_count = *data_ptr++;
				printLog("Received sample #%d", sample_count);
			}
				
			//Chop up data and send to modules with enabled sensors
			if (data_type == SS_DATATYPE_RAW || data_type == SS_DATATYPE_BOTH) {
				for (int i = 0; i < SS_MAX_SUPPORTED_SENSOR_NUM; i++) {
					if (sensor_enabled_mode[i])
					{
						assert_msg(sensor_data_reqs[i], "no ss_data_req found for enabled sensor");
						//??Jason sensor_data_reqs[i]->callback(data_ptr);
						data_ptr += sensor_data_reqs[i]->data_size;
					}
				}
			}
			if (data_type == SS_DATATYPE_ALGO || data_type == SS_DATATYPE_BOTH) {
				for (int i = 0; i < SS_MAX_SUPPORTED_ALGO_NUM; i++) {
					if (algo_enabled_mode[i]) {
						assert_msg(algo_data_reqs[i], 
								"no ss_data_req found for enabled algo");
						//??Jason algo_data_reqs[i]->callback(data_ptr);
						data_ptr += algo_data_reqs[i]->data_size;
					}
				}
			}
		}
	}
    else
    {
#ifdef DEBUG_TIMING_PROFILE
       printf("\r\n");
#endif        
    }
	ss_enable_irq();
	//irq_evt.stop();
}

void ss_clear_interrupt_flag(void){
	m_irq_received_ = false;
}

void irq_handler(void)
{
	m_irq_received_ = true;
}

void ss_irq_handler_selftest(void){
	mfio_int_happened = true;
}

bool ss_reset_mfio_irq(void){
	bool ret = mfio_int_happened;
	mfio_int_happened = false;
	ss_disable_irq();
#ifndef BPT_POLL_MODE
	irq_pin.fall(callback(this, &SSInterface::irq_handler));
#endif
	ss_enable_irq();
	return ret;
}


SS_STATUS set_shut_down_enter(void)
{

	printLog(" ----> entering shut-down backup \r\n ");

	uint8_t cmd_bytes[] = { SS_FAM_W_MODE, 0x00, 0x01 };
	SS_STATUS status = write_cmd2(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes), 0, 0, SS_DEFAULT_CMD_SLEEP_MS);

	return status;
}

SS_STATUS set_shut_down_exit(void)
{

	printf(" ----> exiting shut-down backup \r\n ");
#ifdef JASON
	reset_pin.output();
	reset_pin.write(1);
	wait_ms(1);
	reset_pin.write(0);
	wait_ms(1);
	reset_pin.write(1);
	wait_ms(SS_STARTUP_TO_MAIN_APP_TIME);
	reset_pin.input();
#endif
	SS_STATUS status = SS_SUCCESS;
	return status;
}

SS_STATUS get_dhparams( uint8_t *response, int response_sz ){

	uint8_t cmd_bytes[] = { SH_FAM_R_DHPARAMS , (uint8_t) 0x00 };

	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
								0, 0,
								response, response_sz, SS_DEFAULT_CMD_SLEEP_MS);

	return status;
}

SS_STATUS set_dhlocalpublic(  uint8_t *publicKey , int public_sz ){

	uint8_t cmd_bytes[] = { SH_FAM_W_DHPUBLICKEY, (uint8_t) 0x00 };
	SS_STATUS status = write_cmd2(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
								  publicKey, public_sz, SS_DEFAULT_CMD_SLEEP_MS);
	return status;

}

SS_STATUS get_dhremotepublic( uint8_t *response, int response_sz ){

	uint8_t cmd_bytes[] = { SH_FAM_R_DHPUBLICKEY , (uint8_t) 0x00 };

	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
											0, 0,
											response, response_sz, SS_DEFAULT_CMD_SLEEP_MS);

	return status;
}

SS_STATUS get_authentication( uint8_t *response, int response_sz )
{

//	const int auth_cfg_sz = 32; // fixed to 32 bytes

	uint8_t cmd_bytes[] = { SH_FAM_R_SHADIGEST , (uint8_t) 0x00 };

	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
								0, 0,
								response, response_sz, SS_DEFAULT_CMD_SLEEP_MS);

	return status;
}

SS_STATUS set_i2c_addr(uint8_t addr)
{
	uint8_t cmd_bytes[] = { SS_FAM_W_COMMCHAN, SS_CMDIDX_I2C_ADDR };
	uint8_t data_bytes[] = { (uint8_t)addr };

	SS_STATUS status = write_cmd2(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
								&data_bytes[0], ARRAY_SIZE(data_bytes), SS_DEFAULT_CMD_SLEEP_MS);
	return status;
}

