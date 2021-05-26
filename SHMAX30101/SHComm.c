
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

#include "em_gpio.h"
#include "global.h"
#include "V3.h"
#include "app.h"
#include "SHComm.h"
#include "I2C.h"
#include <stdio.h>
#include <string.h>
#include "sl_sleeptimer.h"

#define SS_I2C_8BIT_SLAVE_ADDR      0xAA
#define SENSORHUB_I2C_ADRESS        SS_I2C_8BIT_SLAVE_ADDR

#define ENABLED   ((int)(1))
#define DISABLED  ((int)(0))

#define SS_DUMP_REG_SLEEP_MS        (100)
#define SS_ENABLE_SENSOR_SLEEP_MS   (40)
#define SS_ENABLE_ALGO_SLEEP_MS     (100)                                    // 500 ---> 50, 2021.03.02
#define SS_DEFAULT_CMD_SLEEP_MS     (2)
#define SS_WAIT_BETWEEN_TRIES_MS    (2)
#define SS_CMD_WAIT_PULLTRANS_MS    (5)
#define SS_FEEDFIFO_CMD_SLEEP_MS	(30)


#define SS_DEFAULT_RETRIES       ((int) (4))
#define SS_ZERO_DELAY               0
#define SS_ZERO_BYTES               0


/*define sample size of algorithm and raw sensor data in bytes*/
#define SH_ALGO_WHRM_SAMPLE_DATABYTES  4
#define SH_ALGO_SP02_SAMPLE_DATABYTES  4


/*define command sequences given in Maxim ME32664 user manual*/
#define SH_GET_HUB_STATUS_CMDSEQ                            {0x00,0x00}
#define SH_SET_OPERATING_MODE_CMDSEQ(opMode)                {0x01,0x00,opMode}
#define SH_SET_OPERATING_MODE_BOOTLOADER_CMDSEQ             {0x02,0x00,0x08}
#define SH_SET_OPERATING_MODE_APPLICATION_CMDSEQ            {0x02,0x00,0x00}
#define SH_SET_OPERATING_MODE_RESET_CMDSEQ                  {0x02,0x00,0x02}
#define SH_GET_OPERATING_MODE_CMDSEQ                        {0x02,0x00}

#define SH_SET_OUTPUT_MODE_CMDSEQ( outMode)                 {0x10,0x00, outMode}
#define SH_SET_OUTMODE_NODATA_CMDSEQ                        {0x10,0x00,0x00}
#define SH_SET_OUTMODE_SENSORDATA_CMDSEQ                    {0x10,0x00,0x01}
#define SH_SET_OUTMODE_ALGODATA_CMDSEQ                      {0x10,0x00,0x02}
#define SH_SET_OUTMODE_PAUSE_CMDSEQ                         {0x10,0x00,0x04}
#define SH_SET_OUTMODE_SENSAMPLECNT_CMDSEQ                  {0x10,0x00,0x05}
#define SH_SET_OUTMODE_ALGOSAMPLECNT_CMDSEQ                 {0x10,0x00,0x06}
#define SH_SET_OUTMODE_ALGOSENSAMPLECNT_CMDSEQ              {0x10,0x00,0x07}

#define SH_GET_OUTPUT_MODE_CMDSEQ                           {0x11,0x00}

#define SH_DFIFO_SET_INT_THRESHOLD_CMDSEQ( ucThreshold )    {0x10,0x01,ucThreshold}
#define SH_DFIFO_GET_INT_THRESHOLD_CMDSEQ                   {0x11,0x01}

#define SH_DFIFO_GET_NSAMPLES_CMDSEQ                        {0x12,0x00}
#define SH_DFIFO_PULL_SAMPLE_CMDSEQ                         {0x12,0x01}
#define SH_GET_EXTINPUT_FIFOSZ_CMDSEQ                       {0x13,0x01}
#define SH_GET_SAMPLEBYTECNT_INPUTFIFO_CMDSEQ               {0x13,0x04}
#define SH_FEED_TO_INPUTFIFO_CMDSEQ                         {0x14,0x00}

#define SH_WRITE_SENSORREG_CMDSEQ( sensorIdx , regAddr )    { 0x40, sensorIdx , regAddr}
#define SH_READ_SENSORREG_CMDSEQ( sensorIdx , regAddr )     { 0x41, sensorIdx , regAddr}
#define SH_READ_AFE_ATTRIBUTES_CMDSEQ(sensorIdx)            { 0x42, sensorIdx}
#define SH_READ_ALLREGISTERS_CMDSEQ(sensorIdx)              { 0x43, sensorIdx}

#define SH_ENABLE_SENSOR_CMDSEQ(sensorIdx , extMode)        {0x44, sensorIdx, 0x01 , extMode }
#define SH_DISABLE_SENSOR_CMDSEQ(sensorIdx)                 {0x44, sensorIdx, 0x00}


#define SH_AGC_SET_ADCRANGE_CMDSEQ( uiPercentage)           {0x50, 0x00, 0x00 , uiPercentage}
#define SH_AGC_SET_STEPSZ_CMDSEQ( uiPercentage)             {0x50, 0x00, 0x01 , uiPercentage}
#define SH_AGC_SET_SENSITIVITY_CMDSEQ( uiPercentage)        {0x50, 0x00, 0x02 , uiPercentage}
#define SH_AGC_SET_NSAMPLESAVRAGING_CMDSEQ( ucNsamples)     {0x50, 0x00, 0x03 , uiNsamples}
#define SH_WHRM_SET_SAMPRATE_CMDSEQ( ucNsamples)            {0x50, 0x02, 0x03 , uiNsamples}


#define SH_ENABLE_ALGO_CMDSEQ( algoIdx)                     { 0x52, algoIdx , 0x01}
#define SH_DISABLE_ALGO_CMDSEQ( algoIdx)                    { 0x52, algoIdx , 0x00}

#define SH_SET_ALGO_CONFIGURATION_CMDSEQ( algoIdx, cgfIdx)  { 0x50 , algoIdx, cgfIdx }
#define SH_GET_ALGO_CONFIGURATION_CMDSEQ( algoIdx, cgfIdx)  { 0x51 , algoIdx, cgfIdx }

#define SH_COMM_CHECK_CMDSEQ                                {0xFF, 0x00}



//phase2 additions
#define SH_CHECKIF_BOOTLDRMODE_CMDSEQ                       { 0x02, 0x00 }
#define SH_SELFTEST_CMDSEQ(idx)                             { 0x70, (uint8_t)idx }
#define SH_EXIT_BOOTLDRMODE_CMDSEQ                          { 0x01, 0x00 }
#define SH_GETLOGSIZE_CMDSEQ                                { 0x90, 0x01 }
#define SH_READHUBLOGS_CMDSEQ                               { 0x90, 0x00 }

#define SH_GET_BOOTLDRPAGESIZE_CMDSEQ                       { 0x81, 0x01 }
#define SH_SET_BOOTLDRPAGECOUNT_CMDSEQ                      { 0x80, 0x02 }

#define BOOTLOADER_MAX_PAGE_SIZE                            8192

/* BOOTLOADER HOST */
#define EBL_CMD_TRIGGER_MODE	                            0
#define EBL_GPIO_TRIGGER_MODE	                            1

//static uint8_t isHubCommInited = 0;

/*
 * SSI API funcions
 * NOTE: Generic functions for any platform.
 *       exceptions: below needs needs modification according to platform and HAL drivers
 *       1. Hard reset function
 *       2. Enable/disable mfio event interrput
 *       3. mfio pin interrupt routine
 *
 * **/

/*global buffer for sensor i2c commands+data*/
uint8_t sh_write_buf[512 + 32];                                               // Buffer size changed from 512 to (512 + 128), 2021.02.25
/*static*/ volatile bool m_irq_received    = false;
static volatile bool mfio_int_happened = false;

/* sensor hub states */
static bool sc_en     = false;
/*static*/ int data_type  = 0;
static int is_sensor_enabled[SS_MAX_SUPPORTED_SENSOR_NUM] = {0};
static int is_algo_enabled[SS_MAX_SUPPORTED_ALGO_NUM]     = {0};
static int enabled_algo_mode[SS_MAX_SUPPORTED_ALGO_NUM]   = {0};
static int sensor_sample_sz[SS_MAX_SUPPORTED_SENSOR_NUM]  = {0};
static int algo_sample_sz[SS_MAX_SUPPORTED_ALGO_NUM]      = {0};

/* Mode to control sesnor hub resets. ie via GPIO based hard reset or Command based soft reset*/
static uint8_t ebl_mode = EBL_GPIO_TRIGGER_MODE;


/* desc  :
 *         Func to init master i2c hardware comm interface with sennor hub
 *                 init mfio interrupt pin and attach irq to pin
 *                 init reset pin
 * params:
 *         N/A
 */
#define PIN_INPUT    1
#define PIN_OUTPUT   0

#if 0
bool sh_get_mfio(void)
{
	return ExpGetMFIO();
}

void reset_pin_input(void)
{
	ExpSetPinsInput(RESET_BIT);                                         //Jason
}

void reset_pin_mode_PullUp(void)
{

}

void mfio_pin_input(void)
{
	ExpSetPinsInput(MFIO_BIT);                                         //Jason
}

void mfio_pin_mode_PullUp(void)
{

}

void irq_pin_enable_irq(void)
{
	ExpSetIRQ_Enable();
}

void irq_pin_disable_irq(void)
{
	ExpSetIRQ_Disable();
}

void reset_pin_write(bool value)
{
	if(value == 0)
		ExpResetPins_2(RESET_BIT);
	else
		ExpSetPins_2(RESET_BIT);
}

void mfio_pin_write(bool value)
{
	if(value == 0)
		ExpResetPins_2(MFIO_BIT);
	else
		ExpSetPins_2(MFIO_BIT);
}

void reset_pin_output(void)
{
	ExpSetPinsOutput(RESET_BIT);
}

void mfio_pin_output(void)
{
	ExpSetPinsOutput(MFIO_BIT);
}
#endif

#if 0
static void delay_1ms(void)
{
    uint32_t count_1ms = 0;

  //while(count_1ms++ < 35800);
	while(count_1ms < 6000)
	{
		count_1ms++;
	}
}
#endif

void wait_ms(uint16_t wait_ms)
{
	//if(v3status.spp == 2)
	//{
	//  sl_sleeptimer_delay_millisecond(wait_ms);
	//}
	//else
#if 0
	{
	  uint16_t ms_count = 0;
	  if(wait_ms == 0) return;
	  while(ms_count < wait_ms)
	  {
		delay_1ms();
		ms_count++;
	  }
	}
#else
	uint32_t delay = sl_sleeptimer_ms_to_tick(wait_ms);
	uint32_t curren_tick1 = sl_sleeptimer_get_tick_count();
	uint32_t diff = 0;
	while(1)
	{
		diff = sl_sleeptimer_get_tick_count() - curren_tick1;
	    if(diff > delay)
	    	return;
	}
#endif
}

void sh_irq_handler(void)
{
  m_irq_received = true;
}

#define BSP_GPIO_PC4_PORT       gpioPortC
#define BSP_GPIO_PC4_PIN        4
#define BSP_GPIO_PC5_PORT       gpioPortC
#define BSP_GPIO_PC5_PIN        5

/**************************************************************************//**
 * @brief Setup GPIO interrupt for pushbuttons.
 *****************************************************************************/
#define USING_INT_PC5
static void mfioGPIOSetup(void)
{
  /* Configure GPIO Clock */
//CMU_ClockEnable(cmuClock_GPIO, true);
#ifdef USING_INT_PC4
  /* Configure Button PC4 as input and enable interrupt */
  GPIO_PinModeSet(BSP_GPIO_PC4_PORT, BSP_GPIO_PC4_PIN, gpioModeInputPull, 1);
  GPIO_ExtIntConfig(BSP_GPIO_PC4_PORT,
                    BSP_GPIO_PC4_PIN,
                    BSP_GPIO_PC4_PIN,          // Interrupt Number
                    false,                     // RisingEdge  Disable
                    true,                      // FallingEdge Enable
                    true                       // interrupt   Enable
					);

  /* Enable EVEN interrupt to catch button press that changes slew rate */
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  NVIC_DisableIRQ(GPIO_EVEN_IRQn);
#else
  /* Configure Button PB1 as input and enable interrupt */
  GPIO_PinModeSet(BSP_GPIO_PC5_PORT, BSP_GPIO_PC5_PIN, gpioModeInputPull, 1);
  GPIO_ExtIntConfig(BSP_GPIO_PC5_PORT,
                    BSP_GPIO_PC5_PIN,
                    BSP_GPIO_PC5_PIN,
                    false,
                    true,
                    true);

  /* Enable ODD interrupt to catch button press that changes slew rate */
  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
  NVIC_DisableIRQ(GPIO_ODD_IRQn);
#endif
}

#ifdef USING_INT_PC4
/**************************************************************************//**
 * @brief GPIO Interrupt handler for even pins.
 *****************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  /* Get and clear all pending GPIO interrupts */
  uint32_t interruptMask = GPIO_IntGet();
  GPIO_IntClear(interruptMask);

  /* Check if button 0 was pressed */
  if (interruptMask & (1 << BSP_GPIO_PC4_PIN))
  {
	  sh_irq_handler();
  }
}
#else
/**************************************************************************//**
 * @brief GPIO Interrupt handler for even pins.
 *****************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  /* Get and clear all pending GPIO interrupts */
  uint32_t interruptMask = GPIO_IntGet();
  GPIO_IntClear(interruptMask);

  /* Check if button 1 was pressed */
  if (interruptMask & (1 << BSP_GPIO_PC5_PIN))
  {
	  sh_irq_handler();
  }
}
#endif

void sh_init_hwcomm_interface(){
#if 1
	mfioGPIOSetup();
  //irq_pin_fall();
#elif 0
	reset_pin_input();
	reset_pin_mode_PullUp();
	mfio_pin_input();                   /*set mfio as input for getting mfio event reporting when sesnor hub is on  application mode */
	mfio_pin_mode_PullUp();
	irq_pin_fall();                     /*attach falling edge interrupt to mfio pin for mfio event reporting */
#else
	reset_pin_output();
	mfio_pin_output();

	reset_pin_write(0);
	mfio_pin_write(1);
	wait_ms(10);
	reset_pin_write(1);

	wait_ms(2000);

	reset_pin_input();
	reset_pin_mode_PullUp();
	mfio_pin_input();                   /*set mfio as input for getting mfio event reporting when sesnor hub is on  application mode */
	mfio_pin_mode_PullUp();

	irq_pin_fall();                    //sh_irq_handler);    /*attach falling edge interrupt to mfio pin for mfio event reporting */
#endif
  //isHubCommInited = 1;
    return;
}

/* mfio pin event reporting related interrupt functions*/
/*
 * data ready event reporting isr from sensor hub
 *
 * params:
 *         N/A
 * */

void sh_clear_mfio_event_flag(void){
	m_irq_received = false;
}

bool sh_has_mfio_event(void){
	return m_irq_received;
}

/*  desc:
 *       func to enable event reporting from sensor hub
 *
 *  params:
 *       N/A
 * */
void sh_enable_irq_mfioevent(void)
{
#if 0
	irq_pin_enable_irq();
#else
  #ifdef USING_INT_PC4
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  #else
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
  #endif
#endif
}

/*  desc:
 *       func to disable event reporting from sensor hub
 *
 *  params:
 *       N/A
 * */
void sh_disable_irq_mfioevent(void)
{
#if 0
	irq_pin_disable_irq();
#else
  #ifdef USING_INT_PC4
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);
  #else
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
  #endif
#endif
}

/* desc:
 *     reset event reporting process from sensor hub, on host side
 *
 *  params:
 *       N/A
 **/
bool sh_reset_mfio_irq(void){
	bool ret = mfio_int_happened;
	mfio_int_happened = false;
	sh_disable_irq_mfioevent();
	//irq_pin.fall(sh_irq_handler);
	sh_enable_irq_mfioevent();
	return ret;
}

void irq_handler_selftest(void){
	mfio_int_happened = true;
}

void sh_mfio_selftest(void){
	sh_disable_irq_mfioevent();
  //irq_pin.fall(irq_handler_selftest);
	sh_enable_irq_mfioevent();
}

static bool in_bootldr;


int in_bootldr_mode(void)
{
	uint8_t cmd_bytes[] = { SS_FAM_R_MODE, SS_CMDIDX_MODE };
	uint8_t rxbuf[2] = { 0 };

	int status = sh_read_cmd(&cmd_bytes[0], sizeof(cmd_bytes),
			0, 0,
			&rxbuf[0], sizeof(rxbuf), SS_DEFAULT_CMD_SLEEP_MS);
	if (status != SS_SUCCESS)
		return -1;

	return (rxbuf[1] & SS_MASK_MODE_BOOTLDR);
}

int exit_from_bootloader(void)
{
	uint8_t cmd_bytes[] = { SS_FAM_W_MODE, SS_CMDIDX_MODE };
	uint8_t data[] = { 0x00 };

	int status = sh_write_cmd_with_data( &cmd_bytes[0], sizeof(cmd_bytes),
										 &data[0], 1 /*sizeof(data)*/,
										 10*SS_DEFAULT_CMD_SLEEP_MS);

	in_bootldr = (status == SS_SUCCESS) ? true : false;

	return status;
}

int stay_in_bootloader(void)
{
	uint8_t cmd_bytes[] = { SS_FAM_W_MODE, SS_CMDIDX_MODE };
	uint8_t data[] = { SS_MASK_MODE_BOOTLDR };

	int status = sh_write_cmd_with_data(
			&cmd_bytes[0], sizeof(cmd_bytes),
			&data[0], sizeof(data), SS_DEFAULT_CMD_SLEEP_MS);

	in_bootldr = (status == SS_SUCCESS) ? true : false;
	return status;
}


static void cfg_mfio(uint8_t dir)
{
	if (dir == PIN_INPUT) {
		mfio_pin_input();
		mfio_pin_mode_PullUp();
	} else {
		sh_enable_irq_mfioevent();
		mfio_pin_output();
	}
}

int sh_reset_to_main_app(void)
{
	int status = -1;
	sh_disable_irq_mfioevent();
	if (ebl_mode == EBL_GPIO_TRIGGER_MODE) {

		reset_pin_output();
		cfg_mfio(PIN_OUTPUT);
		mfio_pin_write(0);
		wait_ms(SS_RESET_TIME - 5);
		reset_pin_write(0);
		wait_ms(SS_RESET_TIME - 5);
		mfio_pin_write(1);
		wait_ms(SS_RESET_TIME - 5);
		reset_pin_write(1);
		//wait_ms(50);
		//mfio_pin.write(0);
		wait_ms(2*SS_STARTUP_TO_MAIN_APP_TIME);
		cfg_mfio(PIN_INPUT);
		reset_pin_input();

    	sh_enable_irq_mfioevent();
		// Verify we exited bootloader mode
		if (in_bootldr_mode() == 0)
			status = SS_SUCCESS;
		else
			status = SS_ERR_UNKNOWN;
	}else{
		status = exit_from_bootloader();
		sh_enable_irq_mfioevent();
	}

	return status;

}

/*
 * desc:
 *    function to init sensor comm interface and get data format.
 *
 * */
void sh_init_hubinterface(void){

	sh_init_hwcomm_interface();
	//sh_get_data_type(&data_type, &sc_en);
    return;
}


/*
 *
 *   SENSOR HUB COMMUNICATION INTERFACE ( Defined in MAX32664 User Guide ) API FUNCTIONS
 *
 *
 * */


//PHASE2 ADDITIONS:

int sh_self_test(int idx, uint8_t *result, int sleep_ms){

	uint8_t cmd_bytes[] = { SS_FAM_R_SELFTEST, (uint8_t)idx }; // = SH_SELFTEST_CMDSEQ;
    uint8_t rxbuf[2];
    result[0] = 0xFF;

    int status = sh_read_cmd(&cmd_bytes[0],sizeof(cmd_bytes) ,
                             0, 0,
						     &rxbuf[0], sizeof(rxbuf),
						     sleep_ms  );

	if (status != SS_SUCCESS)
		return SS_ERR_TRY_AGAIN;

    result[0] = rxbuf[1];
	return status;
}

const char* sh_get_hub_fw_version(void)
{
    uint8_t cmd_bytes[2];
    uint8_t rxbuf[4];

    static char fw_version[32] = "SENSORHUB";

	int bootldr = sh_checkif_bootldr_mode();

	if (bootldr > 0) {
		cmd_bytes[0] = SS_FAM_R_BOOTLOADER;
		cmd_bytes[1] = SS_CMDIDX_BOOTFWVERSION;
		printLog("Bootloader Mode...\r\n");
	} else if (bootldr == 0) {
		cmd_bytes[0] = SS_FAM_R_IDENTITY;
		cmd_bytes[1] = SS_CMDIDX_FWVERSION;
		printLog("Application Mode...\r\n");
	} else {

		return &fw_version[0];
		printLog("Unknown Mode...\r\n");
	}

    int status = sh_read_cmd( &cmd_bytes[0], sizeof(cmd_bytes),
             	 	 	 	  0, 0,
							  &rxbuf[0], sizeof(rxbuf),
							  SS_DEFAULT_CMD_SLEEP_MS );

    if (status == SS_SUCCESS) {
        snprintf(fw_version, sizeof(fw_version),
            "%d.%d.%d", rxbuf[1], rxbuf[2], rxbuf[3]);
	}
    printLog("Hub_fw_version: %s", fw_version);
    printLog("\r\n");

    return &fw_version[0];
}


const char* sh_get_hub_algo_version(void)
{
    uint8_t cmd_bytes[3];
    uint8_t rxbuf[4];

    static char algo_version[64] = "SENSORHUBALGORITHMS";

	int bootldr = sh_checkif_bootldr_mode();

	if (bootldr > 0) {
		cmd_bytes[0] = SS_FAM_R_BOOTLOADER;
		cmd_bytes[1] = SS_CMDIDX_BOOTFWVERSION;
		cmd_bytes[2] = 0;
	} else if (bootldr == 0) {
		cmd_bytes[0] = SS_FAM_R_IDENTITY;
		cmd_bytes[1] = SS_CMDIDX_ALGOVER;
		cmd_bytes[2] = SS_CMDIDX_AVAILSENSORS;
	} else {

		return &algo_version[0];
	}

    int status = sh_read_cmd( &cmd_bytes[0], sizeof(cmd_bytes),
                              0, 0,
                              &rxbuf[0], sizeof(rxbuf),
						      SS_DEFAULT_CMD_SLEEP_MS   );

    if (status == SS_SUCCESS) {
        snprintf(algo_version, sizeof(algo_version),
            "%d.%d.%d", rxbuf[1], rxbuf[2], rxbuf[3]);

    }

    printLog("Hub_algo_version: %s", algo_version);
    printLog("\r\n");

    return &algo_version[0];
}

int sh_send_raw(uint8_t *rawdata, int rawdata_sz)
{
	return sh_write_cmd(&rawdata[0], rawdata_sz, 5 * SS_ENABLE_SENSOR_SLEEP_MS);
}

int sh_get_log_len(int *log_len)
{
	uint8_t cmd_bytes[] = { SS_FAM_R_LOG, SS_CMDIDX_R_LOG_LEN }; // = SH_GETLOGSIZE_CMDSEQ;
	uint8_t rxbuf[2] = {0};
    int logLen = 0;

	int status = sh_read_cmd(&cmd_bytes[0], sizeof(cmd_bytes),
								   0, 0,
								   &rxbuf[0], sizeof(rxbuf),
								   SS_DEFAULT_CMD_SLEEP_MS   );

	if (status == SS_SUCCESS) {
		logLen = (rxbuf[1] << 8) | rxbuf[0];
	}
	*log_len = logLen;

	return status;
}

int sh_read_ss_log(int num_bytes, uint8_t *log_buf, int log_buf_sz)
{
	int bytes_to_read = num_bytes + 1; //+1 for status byte
	//mxm_assert_msg((bytes_to_read <= log_buf_sz), "log_buf too small");

	uint8_t cmd_bytes[] = { SS_FAM_R_LOG, SS_CMDIDX_R_LOG_DATA }; // = SH_READHUBLOGS_CMDSEQ;

	int status = sh_read_cmd(&cmd_bytes[0], sizeof(cmd_bytes),
						     0, 0,
							 log_buf, bytes_to_read,
							 SS_CMD_WAIT_PULLTRANS_MS  );

	return status;
}

// END OF PHASE2 ADDITIONS




int sh_write_cmd( uint8_t *tx_buf,
		          int tx_len,
				  int sleep_ms)
{
	int retries = SS_DEFAULT_RETRIES;
	int ret = m_i2cBus_write(SS_I2C_8BIT_SLAVE_ADDR, (U8*)tx_buf, tx_len, false);
	while (ret != 0 && retries-- > 0) {

		wait_ms(1);
    	ret = m_i2cBus_write(SS_I2C_8BIT_SLAVE_ADDR, (U8*)tx_buf, tx_len, false);
	}
    if(ret != 0)
       return SS_ERR_UNAVAILABLE;

    wait_ms(sleep_ms);

    char status_byte;
    ret = m_i2cBus_read(SS_I2C_8BIT_SLAVE_ADDR, (U8*)&status_byte, 1);
	bool try_again = (status_byte == SS_ERR_TRY_AGAIN);
	while ((ret != 0 || try_again) && retries-- > 0)
	{
	 	wait_ms(sleep_ms);
    	ret = m_i2cBus_read(SS_I2C_8BIT_SLAVE_ADDR, (U8*)&status_byte, 1);
		try_again = (status_byte == SS_ERR_TRY_AGAIN);
	}

    if(ret != 0 || try_again)
    {
        return SS_ERR_UNAVAILABLE;
    }

	return (int) (SS_STATUS)status_byte;
}


int sh_write_cmd_with_data(uint8_t *cmd_bytes,
		                   int cmd_bytes_len,
                           uint8_t *data,
						   int data_len,
                           int cmd_delay_ms)
{
    memcpy(sh_write_buf, cmd_bytes, cmd_bytes_len);
    memcpy(sh_write_buf + cmd_bytes_len, data, data_len);
    int status = sh_write_cmd(sh_write_buf,cmd_bytes_len + data_len, cmd_delay_ms);
    return status;
}


int sh_read_cmd( uint8_t *cmd_bytes,
		         int cmd_bytes_len,
	             uint8_t *data,
				 int data_len,
	             uint8_t *rxbuf,
				 int rxbuf_sz,
                 int sleep_ms )
{

	int retries = SS_DEFAULT_RETRIES;

    int ret = m_i2cBus_write(SS_I2C_8BIT_SLAVE_ADDR, (U8*)cmd_bytes, cmd_bytes_len, (data_len != 0));
    if (data_len != 0)
    {
        ret |= m_i2cBus_write(SS_I2C_8BIT_SLAVE_ADDR, (U8*)data, data_len, false);
    }


	while (ret != 0 && retries-- > 0)
	{
		wait_ms(1);
    	ret = m_i2cBus_write(SS_I2C_8BIT_SLAVE_ADDR, (U8*)cmd_bytes, cmd_bytes_len, (data_len != 0));
	    if (data_len != 0)
	        ret |= m_i2cBus_write(SS_I2C_8BIT_SLAVE_ADDR, (U8*)data, data_len, false);

	}
    if (ret != 0)
    	return SS_ERR_UNAVAILABLE;


    wait_ms(sleep_ms);

    ret = m_i2cBus_read(SS_I2C_8BIT_SLAVE_ADDR, (U8*)rxbuf, rxbuf_sz);
	bool try_again = (rxbuf[0] == SS_ERR_TRY_AGAIN);
	while ((ret != 0 || try_again) && retries-- > 0) {
		wait_ms(sleep_ms);
    	ret = m_i2cBus_read(SS_I2C_8BIT_SLAVE_ADDR, (U8*)rxbuf, rxbuf_sz);
		try_again = (rxbuf[0] == SS_ERR_TRY_AGAIN);
	}
    if (ret != 0 || try_again)
        return SS_ERR_UNAVAILABLE;

    return (int) ((SS_STATUS)rxbuf[0]);
}



int sh_get_sensorhub_status(uint8_t *hubStatus){

	uint8_t ByteSeq[] = SH_GET_HUB_STATUS_CMDSEQ;
	uint8_t rxbuf[2] = { 0 };

	int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
			                    0, 0,
			                    &rxbuf[0], sizeof(rxbuf),
								SS_DEFAULT_CMD_SLEEP_MS);

	*hubStatus = rxbuf[1];
	return status;
}


int sh_get_sensorhub_operating_mode(uint8_t *hubMode){

	uint8_t ByteSeq[] = SH_GET_OPERATING_MODE_CMDSEQ;
	uint8_t rxbuf[2] = { 0 };

	int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
			                    0, 0,
			                    &rxbuf[0], sizeof(rxbuf),
								SS_DEFAULT_CMD_SLEEP_MS);

	*hubMode = rxbuf[1];
	return status;
}


int sh_set_sensorhub_operating_mode(uint8_t hubMode){

	uint8_t ByteSeq[] = SH_SET_OPERATING_MODE_CMDSEQ(hubMode);
	int status = sh_write_cmd( &ByteSeq[0],sizeof(ByteSeq), SS_DEFAULT_CMD_SLEEP_MS);
    return status;

}


//int sh_set_data_type( uint8_t outMode)
int sh_set_data_type(int data_type_, bool sc_en_)
{

#if 0
	uint8_t dataTypeSc = (uint8_t)((sc_en ? SS_MASK_OUTPUTMODE_SC_EN : 0) | ((data_type << SS_SHIFT_OUTPUTMODE_DATATYPE) & SS_MASK_OUTPUTMODE_DATATYPE));
	uint8_t ByteSeq[] = SH_SET_OUTPUT_MODE_CMDSEQ( dataTypeSc);
	int status = sh_write_cmd( &ByteSeq[0],sizeof(ByteSeq), SS_DEFAULT_CMD_SLEEP_MS);
    if( status == 0x00){
    	data_type = data_type_;
        sc_en = sc_en_;
    }
#endif

	uint8_t cmd_bytes[] = { SS_FAM_W_COMMCHAN, SS_CMDIDX_OUTPUTMODE };
	uint8_t data_bytes[] = { (uint8_t)((sc_en_ ? SS_MASK_OUTPUTMODE_SC_EN : 0) |
							((data_type_ << SS_SHIFT_OUTPUTMODE_DATATYPE) & SS_MASK_OUTPUTMODE_DATATYPE)) };

	int status = sh_write_cmd_with_data(&cmd_bytes[0], sizeof(cmd_bytes),
								&data_bytes[0], sizeof(data_bytes),
								SS_DEFAULT_CMD_SLEEP_MS);
	data_type = data_type_;
	sc_en = sc_en_;

	return status;
}


int sh_get_data_type(int *data_type_, bool *sc_en_){

	uint8_t ByteSeq[] = SH_GET_OUTPUT_MODE_CMDSEQ;
	uint8_t rxbuf[2] = {0};
	int status = sh_read_cmd( &ByteSeq[0], sizeof(ByteSeq),
							  0, 0,
							  &rxbuf[0], sizeof(rxbuf),
							  SS_DEFAULT_CMD_SLEEP_MS);
	if (status == 0x00 /*SS_SUCCESS*/) {
		*data_type_ =
			(rxbuf[1] & SS_MASK_OUTPUTMODE_DATATYPE) >> SS_SHIFT_OUTPUTMODE_DATATYPE;
		*sc_en_ =
			(bool)((rxbuf[1] & SS_MASK_OUTPUTMODE_SC_EN) >> SS_SHIFT_OUTPUTMODE_SC_EN);

	}

	return status;

}


int sh_set_fifo_thresh( int threshold ){

#if 0
	uint8_t ucThresh = (uint8_t) (threshold & 0xFF);
	uint8_t ByteSeq[] = SH_DFIFO_SET_INT_THRESHOLD_CMDSEQ(ucThresh );
	int status = sh_write_cmd( &ByteSeq[0],sizeof(ByteSeq), SS_DEFAULT_CMD_SLEEP_MS);
	return status;
#endif

	uint8_t cmd_bytes[] = { SS_FAM_W_COMMCHAN, SS_CMDIDX_FIFOAFULL };
	uint8_t data_bytes[] = { (uint8_t)threshold };

	int status = sh_write_cmd_with_data(&cmd_bytes[0], sizeof(cmd_bytes),
								&data_bytes[0], sizeof(data_bytes),
								SS_DEFAULT_CMD_SLEEP_MS
	                            );
	return status;

}


int sh_get_fifo_thresh(int *thresh){

	uint8_t ByteSeq[] = SH_DFIFO_GET_INT_THRESHOLD_CMDSEQ;
	uint8_t rxbuf[2] = {0};
	int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
							 0, 0,
							 &rxbuf[0], sizeof(rxbuf),
							 SS_DEFAULT_CMD_SLEEP_MS);

	*thresh = (int) rxbuf[1];

	return status;

}


int sh_ss_comm_check(void){


	uint8_t ByteSeq[] = SH_COMM_CHECK_CMDSEQ;
	uint8_t rxbuf[2];

	int status = sh_read_cmd( &ByteSeq[0], sizeof(ByteSeq),
							  0, 0,
							  &rxbuf[0], sizeof(rxbuf),
							  SS_DEFAULT_CMD_SLEEP_MS );

	int tries = 4;
	while (status == SS_ERR_TRY_AGAIN && tries--) {
		wait_ms(1000);
		status = sh_read_cmd( &ByteSeq[0], sizeof(ByteSeq),
									  0, 0,
									  &rxbuf[0], sizeof(rxbuf),
									  SS_DEFAULT_CMD_SLEEP_MS );

	}

	return status;
}


int sh_num_avail_samples(int *numSamples) {

	 uint8_t ByteSeq[] = SH_DFIFO_GET_NSAMPLES_CMDSEQ;
	 uint8_t rxbuf[2] = {0};

	 int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
							  0, 0,
							  &rxbuf[0], sizeof(rxbuf),
							  1);

	 *numSamples = (int) rxbuf[1];

	 return status;
}


int sh_read_fifo_data( int numSamples,
		               int sampleSize,
		               uint8_t* databuf,
					   int databufSz) {

	int bytes_to_read = numSamples * sampleSize + 1; //+1 for status byte

	uint8_t ByteSeq[] = SH_DFIFO_PULL_SAMPLE_CMDSEQ;
	int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
							 0, 0,
							 databuf, bytes_to_read,
							 10);

	return status;
}


/*
 * desc:
 *        func to read sample size for SmartSensor input FIFO for extrenal accel data
 *
 * params:
 *		  __O sampSize:  size of data sample struct in bytes
 * returns:
 *        1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 *
 **/
int sh_read_input_fifo_samplesz( int *sampSize){

	/* NOT IMPLEMENTED IN SS INTERFACE */
   return 0;
}

/*
 * desc:
 *        func to write data  samples to  SmartSensor input FIFO for extrenal accel data
 *
 * params:
          ...
 * returns:
 *        1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_write_input_fifo( void *arg){

	/* NOT IMPLEMENTED IN SS INTERFACE */
    return 0;
}


int sh_set_reg(int idx, uint8_t addr, uint32_t val, int regSz){

	uint8_t ByteSeq[] = SH_WRITE_SENSORREG_CMDSEQ( ((uint8_t)idx) , addr );
	uint8_t data_bytes[4];
	for (int i = 0; i < regSz; i++) {
		data_bytes[i] = (val >> (8 * (regSz - 1)) & 0xFF);
	}
	int status = sh_write_cmd_with_data( &ByteSeq[0], sizeof(ByteSeq),
							             &data_bytes[0], (uint8_t) regSz,
										 SS_DEFAULT_CMD_SLEEP_MS);

    return status;
}


int sh_get_reg(int idx, uint8_t addr, uint32_t *val){


	uint32_t i32tmp;
	uint8_t ByteSeq[] = SH_READ_AFE_ATTRIBUTES_CMDSEQ(((uint8_t) idx));
	uint8_t rxbuf[3] = {0};

	int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
								0, 0,
							 &rxbuf[0], sizeof(rxbuf),
							 SS_DEFAULT_CMD_SLEEP_MS);


    if(status == 0x00 /* SS_SUCCESS */) {

    	int reg_width = rxbuf[1];
    	uint8_t ByteSeq2[] = SH_READ_SENSORREG_CMDSEQ( ((uint8_t)idx) , addr );
    	uint8_t rxbuf2[5] = {0};
    	status = sh_read_cmd(&ByteSeq2[0], sizeof(ByteSeq2),
    						0, 0,
    						&rxbuf2[0], reg_width + 1,
							SS_DEFAULT_CMD_SLEEP_MS);

    	if (status == 0x00  /* SS_SUCCESS */) {
    		i32tmp = 0;
    		for (int i = 0; i < reg_width; i++) {
    			i32tmp = (i32tmp << 8) | rxbuf2[i + 1];
    		}
            *val = i32tmp;
    	}
     }

    return status;

}


int sh_sensor_enable( int idx , int sensorSampleSz , uint8_t ext_mode ){

	uint8_t ByteSeq[] = SH_ENABLE_SENSOR_CMDSEQ( ((uint8_t) idx) ,  ((uint8_t) ext_mode));
	int status = sh_write_cmd( &ByteSeq[0],sizeof(ByteSeq), 1 * SS_ENABLE_SENSOR_SLEEP_MS);                      // 5  ----> 1, 2021.03.02
	if(status == 0x00){

		is_sensor_enabled[idx] = ENABLED;
		sensor_sample_sz[idx] = sensorSampleSz;
	}
    return status;

}

int sh_sensor_enable02( int idx , int sensorSampleSz , uint8_t ext_mode ){

	uint8_t ByteSeq[] = SH_ENABLE_SENSOR_CMDSEQ( ((uint8_t) idx) ,  ((uint8_t) ext_mode));
	int retries = SS_DEFAULT_RETRIES;
	int tx_len = sizeof(ByteSeq);
	int ret = m_i2cBus_write(SS_I2C_8BIT_SLAVE_ADDR, (U8*)ByteSeq, tx_len, false);
	while (ret != 0 && retries-- > 0) {

		wait_ms(1);
    	ret = m_i2cBus_write(SS_I2C_8BIT_SLAVE_ADDR, (U8*)ByteSeq, tx_len, false);
	}
    if(ret != 0)
       return SS_ERR_UNAVAILABLE;
    return 0x00;
#if 0
    wait_ms(1 * SS_ENABLE_SENSOR_SLEEP_MS);                                                                       // 5  ----> 1, 2021.03.04

    char status_byte;
    ret = m_i2cBus_read(SS_I2C_8BIT_SLAVE_ADDR, (U8*)&status_byte, 1);
	bool try_again = (status_byte == SS_ERR_TRY_AGAIN);
	while ((ret != 0 || try_again) && retries-- > 0)
	{
	 	wait_ms(1 * SS_ENABLE_SENSOR_SLEEP_MS);                                                                   // 5  ----> 1, 2021.03.04
    	ret = m_i2cBus_read(SS_I2C_8BIT_SLAVE_ADDR, (U8*)&status_byte, 1);
		try_again = (status_byte == SS_ERR_TRY_AGAIN);
	}

    if(ret != 0 || try_again)
    {
        return SS_ERR_UNAVAILABLE;
    }

	if(status_byte == 0x00){

		is_sensor_enabled[idx] = ENABLED;
		sensor_sample_sz[idx] = sensorSampleSz;
	}
    return status_byte;
#endif
}

int sh_sensor_enable02_status( int idx , int sensorSampleSz , uint8_t ext_mode ){

	//uint8_t ByteSeq[] = SH_ENABLE_SENSOR_CMDSEQ( ((uint8_t) idx) ,  ((uint8_t) ext_mode));
	int retries = SS_DEFAULT_RETRIES;
	//int tx_len = sizeof(ByteSeq);
	int ret = -1;

    char status_byte;
    ret = m_i2cBus_read(SS_I2C_8BIT_SLAVE_ADDR, (U8*)&status_byte, 1);
	bool try_again = (status_byte == SS_ERR_TRY_AGAIN);
	while ((ret != 0 || try_again) && retries-- > 0)
	{
	 	wait_ms(1 * SS_ENABLE_SENSOR_SLEEP_MS);                                                                   // 5  ----> 1, 2021.03.04
    	ret = m_i2cBus_read(SS_I2C_8BIT_SLAVE_ADDR, (U8*)&status_byte, 1);
		try_again = (status_byte == SS_ERR_TRY_AGAIN);
	}

    if(ret != 0 || try_again)
    {
        return SS_ERR_UNAVAILABLE;
    }

	if(status_byte == 0x00){

		is_sensor_enabled[idx] = ENABLED;
		sensor_sample_sz[idx] = sensorSampleSz;
	}
    return status_byte;
}

int sh_sensor_disable( int idx ){

	uint8_t ByteSeq[] = SH_DISABLE_SENSOR_CMDSEQ( ((uint8_t) idx));
	int status = sh_write_cmd( &ByteSeq[0],sizeof(ByteSeq), SS_ENABLE_SENSOR_SLEEP_MS);
	if(status == 0x00){

		is_sensor_enabled[idx] = DISABLED;
	}
	return status;

}


int sh_get_input_fifo_size(int *fifo_size)
{

	uint8_t ByteSeq[] = SH_GET_EXTINPUT_FIFOSZ_CMDSEQ;
	uint8_t rxbuf[3]; /* status + fifo size */


	int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
							  0, 0,
							  rxbuf, sizeof(rxbuf), 2*SS_DEFAULT_CMD_SLEEP_MS);

	*fifo_size = rxbuf[1] << 8 | rxbuf[2];
	return status;
}


int sh_feed_to_input_fifo(uint8_t *tx_buf, int tx_buf_sz, int *nb_written)
{
	int status;

	//uint8_t ByteSeq[] = SH_FEED_TO_INPUTFIFO_CMDSEQ;
	uint8_t rxbuf[3];

	tx_buf[0] = 0x14;
	tx_buf[1] = 0x00;

	status= sh_read_cmd(tx_buf, tx_buf_sz,
			          0, 0,
			          rxbuf, sizeof(rxbuf), SS_FEEDFIFO_CMD_SLEEP_MS);

	*nb_written = rxbuf[1] * 256 + rxbuf[2];
	return status;
}


int sh_get_num_bytes_in_input_fifo(int *fifo_size)
{

    uint8_t ByteSeq[] = SH_GET_SAMPLEBYTECNT_INPUTFIFO_CMDSEQ;
	uint8_t rxbuf[3]; /* status + fifo size */


	int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
							 0, 0,
							 rxbuf, sizeof(rxbuf),
							 2*SS_DEFAULT_CMD_SLEEP_MS);

	*fifo_size = rxbuf[1] << 8 | rxbuf[2];
	return status;
}


/*
 * ALGARITIM RELATED FUNCTIONS :)
 *
 *
 *
 *
 *
 * */


int sh_enable_algo(int idx , int algoSampleSz){

	uint8_t ByteSeq[] =  { 0x52, (uint8_t) idx , 0x01} ;
	int status = sh_write_cmd( &ByteSeq[0],sizeof(ByteSeq), 25 * SS_ENABLE_SENSOR_SLEEP_MS);
	if(status == 0x00){

		is_algo_enabled[idx] = ENABLED;
		algo_sample_sz[idx]  = algoSampleSz;
	}
    return status;

}


int sh_enable_algo_withmode(int idx, int mode, int algoSampleSz)
{

	uint8_t cmd_bytes[] = { SS_FAM_W_ALGOMODE, (uint8_t)idx, (uint8_t)mode };

	int status = sh_write_cmd_with_data(&cmd_bytes[0], sizeof(cmd_bytes), 0, 0, SS_ENABLE_ALGO_SLEEP_MS /*25 * SS_ENABLE_SENSOR_SLEEP_MS*/);

	if (status == SS_SUCCESS) {
		is_algo_enabled[idx]   = ENABLED;
		algo_sample_sz[idx]    = algoSampleSz;
		enabled_algo_mode[idx] = mode;
	}

	return status;
}


int sh_write_cmd_with_data02(uint8_t *cmd_bytes,
		                   int cmd_bytes_len,
                           uint8_t *data,
						   int data_len,
                           int cmd_delay_ms)
{
    memcpy(sh_write_buf, cmd_bytes, cmd_bytes_len);
    memcpy(sh_write_buf + cmd_bytes_len, data, data_len);

	int retries = SS_DEFAULT_RETRIES;
	int ret = m_i2cBus_write(SS_I2C_8BIT_SLAVE_ADDR, (U8*)sh_write_buf, cmd_bytes_len + data_len, false);
	while (ret != 0 && retries-- > 0) {

		wait_ms(1);
    	ret = m_i2cBus_write(SS_I2C_8BIT_SLAVE_ADDR, (U8*)sh_write_buf, cmd_bytes_len + data_len, false);
	}
    if(ret != 0)
       return SS_ERR_UNAVAILABLE;

    wait_ms(cmd_delay_ms);

    char status_byte;
    ret = m_i2cBus_read(SS_I2C_8BIT_SLAVE_ADDR, (U8*)&status_byte, 1);
	bool try_again = (status_byte == SS_ERR_TRY_AGAIN);
	while ((ret != 0 || try_again) && retries-- > 0)
	{
	 	wait_ms(cmd_delay_ms);
    	ret = m_i2cBus_read(SS_I2C_8BIT_SLAVE_ADDR, (U8*)&status_byte, 1);
		try_again = (status_byte == SS_ERR_TRY_AGAIN);
	}

    if(ret != 0 || try_again)
    {
        return SS_ERR_UNAVAILABLE;
    }

	return (int) (SS_STATUS)status_byte;
}


int sh_enable_algo_withmode02(int idx, int mode, int algoSampleSz)
{

	uint8_t cmd_bytes[] = { SS_FAM_W_ALGOMODE, (uint8_t)idx, (uint8_t)mode };

	int cmd_bytes_len = sizeof(cmd_bytes);
    memcpy(sh_write_buf, cmd_bytes, cmd_bytes_len);
  //memcpy(sh_write_buf + cmd_bytes_len, data, data_len);

	int retries = SS_DEFAULT_RETRIES;
	int ret = m_i2cBus_write(SS_I2C_8BIT_SLAVE_ADDR, (U8*)sh_write_buf, cmd_bytes_len + 0, false);
	while (ret != 0 && retries-- > 0) {

		wait_ms(1);
    	ret = m_i2cBus_write(SS_I2C_8BIT_SLAVE_ADDR, (U8*)sh_write_buf, cmd_bytes_len + 0, false);
	}
    if(ret != 0)
       return SS_ERR_UNAVAILABLE;

    return (int) (SS_STATUS)ret;
}

int sh_enable_algo_withmode_status02(int idx, int mode, int algoSampleSz)
{
    char status_byte;
    int retries = SS_DEFAULT_RETRIES;
    int ret = m_i2cBus_read(SS_I2C_8BIT_SLAVE_ADDR, (U8*)&status_byte, 1);
	bool try_again = (status_byte == SS_ERR_TRY_AGAIN);
	while ((ret != 0 || try_again) && retries-- > 0)
	{
	 	wait_ms(SS_ENABLE_ALGO_SLEEP_MS);
    	ret = m_i2cBus_read(SS_I2C_8BIT_SLAVE_ADDR, (U8*)&status_byte, 1);
		try_again = (status_byte == SS_ERR_TRY_AGAIN);
	}

    if(ret != 0 || try_again)
    {
        return SS_ERR_UNAVAILABLE;
    }
	if (status_byte == SS_SUCCESS) {
		is_algo_enabled[idx]   = ENABLED;
		algo_sample_sz[idx]    = algoSampleSz;
		enabled_algo_mode[idx] = mode;
	}
	return (int) (SS_STATUS)status_byte;
}

int sh_disable_algo(int idx){

	uint8_t ByteSeq[] = SH_DISABLE_ALGO_CMDSEQ( ((uint8_t) idx) );
	int status = sh_write_cmd( &ByteSeq[0],sizeof(ByteSeq), SS_ENABLE_SENSOR_SLEEP_MS );
	if(status == 0x00){

		is_algo_enabled[idx] = DISABLED;
	}
    return status;

}


int sh_set_algo_cfg(int algo_idx, int cfg_idx, uint8_t *cfg, int cfg_sz){

	//uint8_t ByteSeq[] = SH_SET_ALGO_CONFIGURATION_CMDSEQ( ((uint8_t) algo_idx) ,  ((uint8_t) cfg_idx)  );

	uint8_t ByteSeq[] = { 0x50 , (uint8_t) algo_idx, (uint8_t) cfg_idx } ;
	int status = sh_write_cmd_with_data( &ByteSeq[0], sizeof(ByteSeq),
			                             cfg, cfg_sz,
										 SS_DEFAULT_CMD_SLEEP_MS);

	return status;

}

int sh_set_algo_cfg_extendedwait(int algo_idx, int cfg_idx, uint8_t *cfg, int cfg_sz , int wait_ms) {

	//uint8_t ByteSeq[] = SH_SET_ALGO_CONFIGURATION_CMDSEQ( ((uint8_t) algo_idx) ,  ((uint8_t) cfg_idx)  );

	uint8_t ByteSeq[] = { 0x50 , (uint8_t) algo_idx, (uint8_t) cfg_idx } ;
	int status = sh_write_cmd_with_data( &ByteSeq[0], sizeof(ByteSeq),
			                             cfg, cfg_sz,
										 wait_ms);

	return status;

}

int sh_get_algo_cfg(int algo_idx, int cfg_idx, uint8_t *cfg, int cfg_sz){

	//uint8_t ByteSeq[] = SH_GET_ALGO_CONFIGURATION_CMDSEQ( ((uint8_t) algo_idx) ,  ((uint8_t) cfg_idx)  );

	uint8_t ByteSeq[] = { 0x51 , (uint8_t) algo_idx, (uint8_t) cfg_idx } ;
	int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
						     0, 0,
							 cfg, cfg_sz,
							 SS_DEFAULT_CMD_SLEEP_MS);
	return status;

}

int sh_get_algo_cfg_extendedwait(int algo_idx, int cfg_idx, uint8_t *cfg, int cfg_sz ,int wait_ms ){

	//uint8_t ByteSeq[] = SH_GET_ALGO_CONFIGURATION_CMDSEQ( ((uint8_t) algo_idx) ,  ((uint8_t) cfg_idx)  );

	uint8_t ByteSeq[] = { 0x51 , (uint8_t) algo_idx, (uint8_t) cfg_idx } ;
	int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
						     0, 0,
							 cfg, cfg_sz,
							 wait_ms );
	return status;

}

/*
 * desc:
 *      func to get active cumulative sample size of sensor hub in order to
 *           calculate number of bytes to be read from sensor hub report data buffer
 *
 * params:
 *      __I data_type : active data type of sensor hub -> no data              :0 (SS_DATATYPE_PAUSE)
 *                                                        raw sensor data only :1 (SS_DATATYPE_RAW)
 *                                                        algo data only       :2 (SS_DATATYPE_ALGO)
 *                                                        algo+raw data        :3 (SS_DATATYPE_BOTH)
 *      __O sample_size : calculated active cumulative sample size

 * returns:
 *        N/A
 *
 **/
/*static*/ void fifo_sample_size(int data_type_, int *sample_size)
{

    int tmpSz = 0;
	//*sample_size = 0;

	if (data_type_ == SS_DATATYPE_RAW || data_type_ == SS_DATATYPE_BOTH) {
		for (int i = 0; i < SS_MAX_SUPPORTED_SENSOR_NUM; i++) {
			if (is_sensor_enabled[i]) {
				tmpSz += sensor_sample_sz[i];
				//*sample_size += sensor_data_reqs[i]->data_size;
			}
		}
	}

	if (data_type_ == SS_DATATYPE_ALGO || data_type_ == SS_DATATYPE_BOTH) {
		for (int i = 0; i < SS_MAX_SUPPORTED_ALGO_NUM; i++) {
			if (is_algo_enabled[i]) {
				tmpSz += algo_sample_sz[i];
				//*sample_size += algo_data_reqs[i]->data_size;
			}
		}
	}

	*sample_size = tmpSz;
}


int sh_ss_execute_once( uint8_t *databuf , int databufLen , int *nSamplesRead){

    if(m_irq_received == false)
	{
		  *nSamplesRead = 0;
		  return -1;
	}

	//uint8_t sample_count;

    sh_disable_irq_mfioevent();
    sh_clear_mfio_event_flag();

	uint8_t hubStatus = 0;
	int status = sh_get_sensorhub_status(&hubStatus);
	if(status != 0x00 /*SS_SUCCESS*/){
    	*nSamplesRead = 0;
    	sh_enable_irq_mfioevent();
        return status;
    }

    if (hubStatus & SS_MASK_STATUS_DATA_RDY) {

    	 int num_samples = 1;
    	 status = sh_num_avail_samples(&num_samples);
      	 if (status != 0x00 /*SS_SUCCESS*/){
    		 *nSamplesRead = 0;
    		 sh_enable_irq_mfioevent();
    		 return status;
         }


    	 int sample_size;
    	 fifo_sample_size(data_type, &sample_size);
    	 /*DEBUG *///


    	 int bytes_to_read = num_samples * sample_size + 1; //+1 for status byte
         if ( bytes_to_read > databufLen) {
 			//Reduce number of samples to read to fit in buffer
 			num_samples = (databufLen - 1) / sample_size;
 		 }


        wait_ms(5);
        status = sh_read_fifo_data(num_samples, sample_size, &databuf[0], databufLen);
        if(status != 0x00 /*SS_SUCCESS*/){
        	*nSamplesRead = 0;
        	sh_enable_irq_mfioevent();
        	return status;
        }
        *nSamplesRead = num_samples;
    }

    sh_enable_irq_mfioevent();
    return status;
}



/*
 * BOOTLOADER RELATED FUNCTIONS
 *
 *
 * */

static const int aes_nonce_sz = 11;
static const int aes_auth_sz  = 16;
static int bl_comm_delay_factor = 1;



int sh_set_bootloader_delayfactor(const int factor ) {

	int status = -1;
	if( factor >= 1  && factor < 51){
	    bl_comm_delay_factor = factor;
	    status = 0x00;
	}

	return status;

}

int sh_get_bootloader_delayfactor(void){

     return bl_comm_delay_factor;
}

int sh_put_in_bootloader(void)
{
	return sh_set_sensorhub_operating_mode( 0x08);
}

int sh_checkif_bootldr_mode(void)
{
	uint8_t hubMode;
	int status = sh_get_sensorhub_operating_mode(&hubMode);
	return (status != SS_SUCCESS)? -1:(hubMode & SS_MASK_MODE_BOOTLDR);
}

int sh_get_bootloader_pagesz(int *pagesz){

	//uint8_t ByteSeq[]= SH_GET_BOOTLDRPAGESIZE_CMDSEQ;
	uint8_t ByteSeq[]= { SS_FAM_R_BOOTLOADER, SS_CMDIDX_PAGESIZE };
    uint8_t rxbuf[3];
    int sz = 0;

    int status = sh_read_cmd( &ByteSeq[0], sizeof(ByteSeq),
                          0, 0,
                          &rxbuf[0], sizeof(rxbuf),
						  SS_DEFAULT_CMD_SLEEP_MS);
    if (status == 0x00) {
           //rxbuf holds page size in big-endian format
            sz = (256*(int)rxbuf[1]) + rxbuf[2];
            if(sz > BOOTLOADER_MAX_PAGE_SIZE ) {
                   sz = -2;
            }
    }

    *pagesz = sz;

    return status;

}

int sh_set_bootloader_numberofpages(const int pageCount){

	//uint8_t ByteSeq[] = SH_SET_BOOTLDRPAGECOUNT_CMDSEQ;
    uint8_t ByteSeq[] = { SS_FAM_W_BOOTLOADER, SS_CMDIDX_SETNUMPAGES };
    //num pages = 256*MSB + LSB
    uint8_t data_bytes[] = { (uint8_t)((pageCount >> 8) & 0xFF), (uint8_t)(pageCount & 0xFF) };

    int status = sh_write_cmd_with_data(&ByteSeq[0], sizeof(ByteSeq),
								        &data_bytes[0], sizeof(data_bytes),
										bl_comm_delay_factor * SS_DEFAULT_CMD_SLEEP_MS );

    return status;

}

int sh_set_bootloader_iv(uint8_t iv_bytes[aes_nonce_sz]){

	 uint8_t ByteSeq[] = { SS_FAM_W_BOOTLOADER, SS_CMDIDX_SETIV };
	 int status = sh_write_cmd_with_data( &ByteSeq[0], sizeof(ByteSeq),
			                              &iv_bytes[0], aes_nonce_sz /*sizeof(iv_bytes)*/,
										  bl_comm_delay_factor * SS_DEFAULT_CMD_SLEEP_MS
										  );

     return status;

}


int sh_set_bootloader_auth(uint8_t auth_bytes[aes_auth_sz]){

	 uint8_t ByteSeq[] = { SS_FAM_W_BOOTLOADER, SS_CMDIDX_SETAUTH };
	 int status = sh_write_cmd_with_data( &ByteSeq[0], sizeof(ByteSeq),
			                              &auth_bytes[0], aes_auth_sz /*sizeof(auth_bytes)*/,
										  bl_comm_delay_factor * SS_DEFAULT_CMD_SLEEP_MS
										  );

     return status;

}


int sh_set_bootloader_erase(void){

    uint8_t ByteSeq[] = { SS_FAM_W_BOOTLOADER, SS_CMDIDX_ERASE };

    int status = sh_write_cmd_with_data(&ByteSeq[0], sizeof(ByteSeq),
                                        0, 0,
										bl_comm_delay_factor * SS_BOOTLOADER_ERASE_DELAY);

    return status;

}


int sh_bootloader_flashpage(uint8_t *flashDataPreceedByCmdBytes , const int page_size){

	static const int flash_cmdbytes_len   = 2;
	static const int check_bytes_len      = 16;
	static const int page_write_time_ms   = 200;

    //static const uint8_t ByteSeq[] = { SS_FAM_W_BOOTLOADER, SS_CMDIDX_SENDPAGE };
    int status = -1;

    if( (*flashDataPreceedByCmdBytes == SS_FAM_W_BOOTLOADER) &&  ( *(flashDataPreceedByCmdBytes+1) == SS_CMDIDX_SENDPAGE ) ) {

		/* We do not use sh_write_cmd_with_data function because internal buffers of the function
		   is limited to 512 bytes which does not support if flashing page size is bigger */
		status = sh_write_cmd(flashDataPreceedByCmdBytes, page_size + check_bytes_len + flash_cmdbytes_len, bl_comm_delay_factor * page_write_time_ms);

    }
	return status;

}


int sh_get_ss_fw_version(uint8_t *fwDesciptor  , uint8_t *descSize)
{

	int status = -1;
	uint8_t cmd_bytes[2];
    uint8_t rxbuf[4];

	int bootldr = in_bootldr_mode();

	if (bootldr > 0) {
		cmd_bytes[0] = SS_FAM_R_BOOTLOADER;
		cmd_bytes[1] = SS_CMDIDX_BOOTFWVERSION;
	} else if (bootldr == 0) {
		cmd_bytes[0] = SS_FAM_R_IDENTITY;
		cmd_bytes[1] = SS_CMDIDX_FWVERSION;
	} else {
		return -1;
	}

    status = sh_read_cmd( &cmd_bytes[0], sizeof(cmd_bytes),
             	 	 	 	 	 	0, 0,
								    &rxbuf[0], sizeof(rxbuf) ,
									SS_DEFAULT_CMD_SLEEP_MS );

    if (status == 0x00 /*SS_SUCCESS*/) {
    	*fwDesciptor       = rxbuf[1];
    	*(fwDesciptor + 1) = rxbuf[2];
    	*(fwDesciptor + 2) = rxbuf[3];
    	*descSize = 3;
    }else{
    	*descSize = 0;
    }

    return status;

}


/*
#ifdef __cplusplus
}
#endif
*/

