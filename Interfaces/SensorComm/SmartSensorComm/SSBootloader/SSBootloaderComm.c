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
#include <string.h>
#include <stdio.h>
#include "SSBootloaderComm.h"
#include "DSInterface.h"
#include "SSInterface.h"
#include "Peripherals.h"
#include "assert.h"
#include "utils.h"
#include "assert.h"

static const char* const cmd_tbl[] = {
    "bootldr",
    "exit",
    "reset",
    "page_size",
    "num_pages",
    "set_iv",
    "set_auth",
    "erase",
    "page_erase",
    "flash",
	"set_cfg bl enter_mode",
	"set_cfg bl enter_pin",
	"set_cfg bl enter_pol",
	"set_cfg bl exit_mode",
	"set_cfg bl exit_to",
	"set_cfg bl save",
};

/* PRIVATE CONST VARIABLES */
#define AES_NONCE_SIZE            11
#define AES_AUTH_SIZE             16
#define MAX_PAGE_SIZE             8192
#define CHECKBYTES_SIZE           16

char bootldr_version[128];

int num_pages;
int page_size;

bool got_page_size;
bool sent_num_pages;

static const char* sensor_type;
static uint8_t     vis;

SSBootloaderComm ssBoot;

static const char* get_type_01(void);
static uint8_t is_visible_01(void);
static uint8_t parse_command_01(const char* cmd);
static void stop_01(void);

static int data_report_execute_01(char* buf, int size)
{
	return 0;
}

void SSBootloaderComm_init(void)
{
	//sensor = NULL;
	sensor_type = "bootLoader";
	vis = false;

	ssBoot.get_type      = (get_type_callback)&get_type_01;
	ssBoot.is_visible    = (is_visible_callback)&is_visible_01;
  //ssBoot.is_enabled    = is_enabled_01;
    ssBoot.stop          = (stop_callback)&stop_01;
	ssBoot.parse_command = (parse_command_callback)&parse_command_01;
	ssBoot.data_report_execute = (data_report_execute_callback)&data_report_execute_01;
}

static void stop_01(void)
{

}


static const char* get_type_01(void)
{
	return sensor_type;
}

static uint8_t is_visible_01(void)
{
	return vis;
}

int parse_iv(const char* cmd, uint8_t* iv_bytes)
{
    char cmdStr[] = "set_iv ";
    int length = strlen(cmd);
    int expected_length = strlen(cmdStr) + 2*AES_NONCE_SIZE;
    if (length != expected_length) {
        pr_err("Couldn't parse IV, incorrect number of characters (len:%d, expected:%d)\n",
               length, expected_length);
        return COMM_INVALID_PARAM;
    }

    const char* ivPtr = cmd + strlen(cmdStr);

    int num_found;
    int byteVal;
    for (int ividx = 0; ividx < AES_NONCE_SIZE; ividx++) {
        num_found = sscanf(ivPtr, "%2X", &byteVal);

        if (num_found != 1 || byteVal > 0xFF) {
            pr_err("Couldn't parse byte %d of IV\n", ividx);
            return COMM_INVALID_PARAM;
        }
        iv_bytes[ividx] = (uint8_t)byteVal;
        ivPtr += 2;
    }

    return COMM_SUCCESS;
}

void flash_page_data(void)
{
    int totalBytes = 0;
    int currentPage = 1;

    static uint8_t tx_buf[MAX_PAGE_SIZE + CHECKBYTES_SIZE + 2] = { SS_FAM_W_BOOTLOADER, SS_CMDIDX_SENDPAGE };
    //uint8_t *data_buffer = &tx_buf[2];

    while (currentPage <= num_pages) {
        pr_info("Waiting for page %d/%d data (%d bytes)...", currentPage, num_pages, page_size);

        //Collect page data + checksum from PC/Android
//		totalBytes = 100;
        while (totalBytes < (page_size + CHECKBYTES_SIZE)) {
            //??data_buffer[totalBytes++] = m_USB->_getc();
        }

        pr_info("Done\r\n");

        //Send data to SmartSensor
        SS_STATUS status = write_cmd(tx_buf, page_size + CHECKBYTES_SIZE + 2, 2000);
		pr_err("status: %d\r\n", status);

        //Inform PC/Andoid of status
        if (status == SS_ERR_BTLDR_CHECKSUM) {
            pr_err("Verify checksum failed!\r\n");
            pr_info("\r\npageFlashDone err=%d\r\n", FLASH_ERR_CHECKSUM);
        } else if (status != SS_SUCCESS) {
            pr_err("Page flash failed!\r\n");
            pr_info("\r\npageFlashDone err=%d\r\n", FLASH_ERR_GENERAL);
        } else {
            currentPage++;
            pr_err("Page flash successful!\r\n");
            pr_info("\r\npageFlashDone err=%d\r\n", COMM_SUCCESS);
        }

        totalBytes = 0;
    }
}

int parse_auth(const char* cmd, uint8_t *auth_bytes)
{
    char cmdStr[] = "set_auth ";
    int length = strlen(cmd);
    int expected_length = strlen(cmdStr) + 2*AES_AUTH_SIZE;
    if (length != expected_length) {
        pr_err("Couldn't parse Auth bytes, incorrect number of characters (len:%d, expected:%d)\n",
            length, expected_length);
        return COMM_INVALID_PARAM;
    }

    const char* macPtr = cmd + strlen(cmdStr);

    int num_found;
    int byteVal;
    for (int aidx = 0; aidx < AES_AUTH_SIZE; aidx++) {
        num_found = sscanf(macPtr, "%2X", &byteVal);

        if (num_found != 1 || byteVal > 0xFF) {
            pr_err("Couldn't parse byte %d of Auth\n", aidx);
            return COMM_INVALID_PARAM;
        }

        auth_bytes[aidx] = (uint8_t)byteVal;
        macPtr += 2;
    }

    return COMM_SUCCESS;
}

static uint8_t parse_command_01(const char* cmd)
{
    int ret = EXIT_SUCCESS;
    bool recognizedCmd = false;

    //if (!ss_int) {
    //    pr_err("No SmartSensor Interface defined!");
    //    return false;
    //}
    //if (!ds_int) {
    //    pr_err("No DeviceStudio Interface defined!");
    //    return false;
    //}

    for (int i = 0; i < NUM_CMDS_TT; i++) {
        if (starts_with(cmd, cmd_tbl[i])) {
            cmd_state_t01 user_cmd = (cmd_state_t01)i;
            recognizedCmd = true;

            switch (user_cmd) {
                case cmd_enter_bootldr:
                {
					SS_STATUS status;
					status = reset_to_bootloader();
					if (status == SS_SUCCESS)
	                    pr_info("\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);
					else
	                    pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);

                    set_fw_platform(get_ss_platform_name());
                    set_fw_version(get_ss_fw_version());
                    got_page_size = false;
                    sent_num_pages = false;
                } break;

                case cmd_exit_bootldr:
                {
					SS_STATUS status = reset_to_main_app();
                    if (status == SS_SUCCESS)
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);
                    else
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);

                    set_fw_platform(get_ss_platform_name());
                    set_fw_version(get_ss_fw_version());

                } break;

                case cmd_reset:
                {
					SS_STATUS status = reset();
                    if (status == SS_SUCCESS)
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);
                    else
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
                    pr_info("\r\n%s err=%d\r\n", cmd, ret);

                    set_fw_platform(get_ss_platform_name());
                    set_fw_version(get_ss_fw_version());

                } break;

                case cmd_page_size:
                {
                    uint8_t cmd_bytes[] = { SS_FAM_R_BOOTLOADER, SS_CMDIDX_PAGESIZE };
                    uint8_t rxbuf[3];

                    SS_STATUS status = read_cmd(
                            &cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
                            0, 0,
                            &rxbuf[0], ARRAY_SIZE(rxbuf), SS_DEFAULT_CMD_SLEEP_MS);

                    if (status == SS_SUCCESS) {
                        //rxbuf holds page size in big-endian format
                        page_size = (256*(int)rxbuf[1]) + rxbuf[2];
                        assert_msg(page_size <= MAX_PAGE_SIZE, "Page size exceeds maximum allowed");

                        pr_info("\r\n%s value=%d err=%d\r\n", cmd, page_size, COMM_SUCCESS);
                        got_page_size = true;

                    } else {
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
                    }
                } break;

                case cmd_num_pages:
                {
                    int num_tok = sscanf(cmd, "num_pages %d", &num_pages);
                    if (num_tok != 1) {
                        pr_info("\r\n%s value=%d err=%d\r\n", cmd, 0, COMM_INVALID_PARAM);
						break;
                    }

                    uint8_t cmd_bytes[] = { SS_FAM_W_BOOTLOADER, SS_CMDIDX_SETNUMPAGES };
                    //num pages = 256*MSB + LSB
                    uint8_t data[] = { (uint8_t)((num_pages >> 8) & 0xFF), (uint8_t)(num_pages & 0xFF) };

                    SS_STATUS status = write_cmd2(
                            &cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
                            &data[0], ARRAY_SIZE(data), SS_DEFAULT_CMD_SLEEP_MS);

                    if (status == SS_SUCCESS) {
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);
                        sent_num_pages = true;

                    } else {
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
                    }
                } break;

                case cmd_set_iv:
                {
                    uint8_t iv_bytes[AES_NONCE_SIZE];
                    ret = parse_iv(cmd, &iv_bytes[0]);
                    if (ret != COMM_SUCCESS) {
                        pr_info("\r\n%s err=%d\r\n", cmd, ret);
                    }
                    else
                    {
                        uint8_t cmd_bytes[] = { SS_FAM_W_BOOTLOADER, SS_CMDIDX_SETIV };

                        SS_STATUS status = write_cmd2(
                            &cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
                            &iv_bytes[0], ARRAY_SIZE(iv_bytes), SS_DEFAULT_CMD_SLEEP_MS);

                        if (status == SS_SUCCESS) {
                            pr_info("\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);

                        } else {
                            pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
                        }
                    }
                } break;

                case cmd_set_auth:
                {
                    uint8_t auth_bytes[AES_AUTH_SIZE];
                    ret = parse_auth(cmd, &auth_bytes[0]);
                    if (ret != COMM_SUCCESS) {
                        pr_info("\r\n%s err=%d\r\n", cmd, ret);
                    }
                    else
                    {
                        uint8_t cmd_bytes[] = { SS_FAM_W_BOOTLOADER, SS_CMDIDX_SETAUTH };

                        SS_STATUS status = write_cmd2(
                            &cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
                            &auth_bytes[0], ARRAY_SIZE(auth_bytes), SS_DEFAULT_CMD_SLEEP_MS);

                        if (status == SS_SUCCESS) {
                            pr_info("\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);

                        } else {
                            pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
                        }
                    }
                } break;

                case cmd_erase:
                {
                    uint8_t cmd_bytes[] = { SS_FAM_W_BOOTLOADER, SS_CMDIDX_ERASE };

                    SS_STATUS status = write_cmd2(
                            &cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
                            0, 0,
							SS_BOOTLOADER_ERASE_DELAY);
                    if (status == SS_SUCCESS)
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);
                    else
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
                } break;

                case cmd_page_erase:
                {
					int page_num_to_erase;
                    int num_tok = sscanf(cmd, "page_erase %d", &page_num_to_erase);
                    if (num_tok != 1) {
                        pr_info("\r\n%s value=%d err=%d\r\n", cmd, 0, COMM_INVALID_PARAM);
						break;
                    }

                    uint8_t cmd_bytes[] = { SS_FAM_W_BOOTLOADER, SS_CMDIDX_ERASE_PAGE };
                    //num pages = 256*MSB + LSB
                    uint8_t data[] = { (uint8_t)((page_num_to_erase >> 8) & 0xFF), (uint8_t)(page_num_to_erase & 0xFF) };

                    SS_STATUS status = write_cmd2(
                            &cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
                            &data[0], ARRAY_SIZE(data), 50);

                    if (status == SS_SUCCESS) {
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);
                        sent_num_pages = true;
                    } else {
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
                    }
                } break;

                case cmd_flash:
                {
                    if (got_page_size && sent_num_pages) {
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);
                        flash_page_data();
                    } else {
                        pr_err("Can't enter flash mode. Need number of pages, and size of page"
                                "(num_pages, page_size, commands)\r\n");
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
                    }
                } break;

				case cmd_setcfg_bl_enter_mode:
				{
					uint8_t mode;
					ret = parse_cmd_data(cmd, cmd_tbl[i], &mode, 1, false);
					if (ret != 1) {
						pr_err("parse_cmd_data=%d\r\n", ret);
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_INVALID_PARAM);
						break;
					}

                    uint8_t cmd_bytes[] = { SS_FAM_W_BOOTLOADER_CFG, SS_CMDIDX_BL_ENTRY, SS_BL_CFG_ENTER_BL_MODE, mode };
					SS_STATUS status = write_cmd2(
                            &cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
                            0, 0, SS_DEFAULT_CMD_SLEEP_MS);
                    if (status == SS_SUCCESS)
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);
                    else
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);

				} break;

				case cmd_setcfg_bl_ebl_pin:
				{
					uint8_t pin[2];
					ret = parse_cmd_data(cmd, cmd_tbl[i], &pin[0], 2, false);
					if (ret != 2) {
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_INVALID_PARAM);
						break;
					}

                    uint8_t cmd_bytes[] = { SS_FAM_W_BOOTLOADER_CFG, SS_CMDIDX_BL_ENTRY, SS_BL_CFG_EBL_PIN,
											pin[0], pin[1]};
					SS_STATUS status = write_cmd2(
                            &cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
                            0, 0, SS_DEFAULT_CMD_SLEEP_MS);
                    if (status == SS_SUCCESS)
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);
                    else
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);

				} break;

				case cmd_setcfg_bl_ebl_pol:
				{
					uint8_t mode;
					ret = parse_cmd_data(cmd, cmd_tbl[i], &mode, 1, false);
					if (ret != 1) {
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_INVALID_PARAM);
						break;
					}

                    uint8_t cmd_bytes[] = { SS_FAM_W_BOOTLOADER_CFG, SS_CMDIDX_BL_ENTRY, SS_BL_CFG_EBL_POL, mode };
					SS_STATUS status = write_cmd2(
                            &cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
                            0, 0, SS_DEFAULT_CMD_SLEEP_MS);
                    if (status == SS_SUCCESS)
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);
                    else
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);

				} break;

				case cmd_setcfg_bl_exit_mode:
				{
					uint8_t mode;
					ret = parse_cmd_data(cmd, cmd_tbl[i], &mode, 1, false);
					if (ret != 1) {
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_INVALID_PARAM);
						break;
					}

                    uint8_t cmd_bytes[] = { SS_FAM_W_BOOTLOADER_CFG, SS_CMDIDX_BL_EXIT, SS_BL_CFG_EXIT_BL_MODE, mode };
					SS_STATUS status = write_cmd2(
                            &cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
                            0, 0, SS_DEFAULT_CMD_SLEEP_MS);
                    if (status == SS_SUCCESS)
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);
                    else
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);

				} break;
				case cmd_setcfg_bl_timeout:
				{
					uint8_t to;
					ret = parse_cmd_data(cmd, cmd_tbl[i], &to, 1, false);
					if (ret != 1) {
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_INVALID_PARAM);
						break;
					}

                    uint8_t cmd_bytes[] = { SS_FAM_W_BOOTLOADER_CFG, SS_CMDIDX_BL_EXIT, SS_BL_CFG_TIMEOUT, to };
					SS_STATUS status = write_cmd2(
                            &cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
                            0, 0, SS_DEFAULT_CMD_SLEEP_MS);
                    if (status == SS_SUCCESS)
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);
                    else
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);

				} break;
				case cmd_setcfg_bl_save:
				{
                    uint8_t cmd_bytes[] = { SS_FAM_W_BOOTLOADER_CFG, SS_CMDIDX_BL_SAVE };

					SS_STATUS status = write_cmd2(
                            &cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
                            0, 0, 50);
                    if (status == SS_SUCCESS)
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);
                    else
                        pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);

				} break;

                default:
                {
                    assert_msg(false, "Invalid switch case!");
                }
            }
        }
    }

    return recognizedCmd;
}
