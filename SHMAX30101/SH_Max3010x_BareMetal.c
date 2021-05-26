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


#include "SH_Max3010x_BareMetal.h"
#include "SHComm.h"
#include <string.h> //for memset
#include <stdint.h>
#include <stdio.h>
#include "global.h"
#include "v3.h"

// Defines
#define SSMAX30101_REG_SIZE        1	//Taken from API doc
#define SSMAX30101_MODE1_DATASIZE  12	//Taken from API doc
#define SSWHRM_MODE1_DATASIZE      6	//Taken from API doc
#define SSACCEL_MODE1_DATASIZE     6	//Taken from API doc
#define SSAGC_MODE1_DATASIZE       0	//Taken from API doc
#define SSBPT_MODE1_2_DATASIZE     17	//Taken from API doc

// sensor configuration
#define ENABLE_MAX30101
#define ENABLE_BPT


const cmd_interface_tp CMDTABLE3010xBPT[] = {

	/* 0*/	{  "get_format bpt_mode"     		, SH_Max3010x_get_bpt_dataformat        ,  "returns BPT algo sample format as bit fields. arg: 0->estimation mode 1-> calib mode as bit fields"					},
	/* 1*/	{  "enable_bpt calibration" 	    , SH_Max3010x_measure_bpt_calibration 	,  "start BPT calibration measurement"																					},
	/* 2*/	{  "enable_bpt estimation" 		    , SH_Max3010x_measure_bpt_estimation	,  "start BPT estimation measurement"																					},
	/* 3*/	{  "get_reg ppgsensor"       		, SH_Max3010x_get_ppgreg                ,  "get register value of 8614x sensor, usage:  get_reg ppgsensor rAddr(1byte)" 										},
	/* 4*/	{  "set_reg ppgsensor"       		, SH_Max3010x_set_ppgreg                ,  "set register value of 8614x sensor, usage :  set_reg ppgsensor rAddr(1byte) rval(1byte)" 							},
	/* 5*/	{  "set_cfg ppg agc"                , SH_Max3010x_set_ppg_agcusage          ,  "on/off ppg automatic gain control for bpt algo,  usage: set_cfg ppg_agc X , X: 0 off 1 on"							},
	/* 6*/	{  "set_cfg bpt med"                , SH_Max3010x_set_bpt_med_config 		,  " usage: set_cfg bpt med medVal(1 byte hex) " 																		},
	/* 7*/	{  "set_cfg bpt sys_bp"             , SH_Max3010x_set_bpt_sysbp_config 		,  " set systolic blood pressure initials. usage: set_cfg bpt sys_bp  110 110 110 (110: integer)" 						},
	/* 8*/	{  "set_cfg bpt dia_bp"             , SH_Max3010x_set_bpt_diabp_config 		,  " set diastolic blood pressure initials. usage: set_cfg bpt sys_bp 70 70 70 (70: integer)"  							},
	/* 9*/	{  "get_cfg bpt cal_result"         , SH_Max3010x_get_bpt_calibration  		,  " get bpt calibration result for user" 																				},
	/*10*/	{  "set_cfg bpt cal_result"         , SH_Max3010x_set_bpt_calibration 		,  " set bpt calibration result for user. usage: set_cfg bpt cal_result ABCD12.... (624x2 characters AB= 1 byte as hex)"},
	/*11*/  {  "set_cfg bpt date_time"          , SH_Max3010x_set_bpt_date_time 		,  " set time of measurement for both calibration and estimatio.: usage: set_cfg bpt date_time 190328 090742 \
			                                                                                 190328 date 090742 time. look ar user manual for date and time data strcutures" 									},
	/*12*/	{  "set_cfg bpt nonrest"            , SH_Max3010x_set_bpt_nonrest_config  	,  " sets resting status of user " 																						},
	/*13*/	{  "set_cfg bpt spo2_coefs"         , SH_Max3010x_set_bpt_spo2_coeffs  		,  " sets predetermined spo2 const coefficients usage: set_cfg bpt spo2_coefs 1234abcd 1234abcd 1234abcd \
																							 params 1234abcd in hex" 																							},
	/*14*/	{  "get_cfg bpt spo2_coefs"         , SH_Max3010x_get_bpt_spo2_coeffs  		,  " gets predetermined spo2 const coefficients used by algorithm" 														},
	/*16*/	{  "stop"                           , SH_Max3010x_stop						,  "stops BPT measurement" 																								},
	/*18*/	{  "set_cfg bpt sys_dia"            , SH_Max3010x_set_bpt_sys_dia_config 	,  " set systolic cal_index, diastolic blood pressure initials. usage: set_cfg bpt sys_dia  0 117 76 (integer)" 						},
	/*19*/	{  "get_cfg bpt cal_index"          , SH_Max3010x_get_bpt_cal_index  		,  " get bpt cal_index for user calibration"},
	/*20*/	{  "set_cfg bpt cal_index"          , SH_Max3010x_set_bpt_cal_index  		,  " set bpt cal_index for user calibration"},
};




// end of senor and algorithm configuration

// end of defines

//function pointer use to perform arithmetic operation
typedef void (*rx_data_callback)(uint8_t *);
typedef struct {
	int data_size;
	rx_data_callback rx_data_parser;
} ss_data_req;


typedef struct Max30101_SH_Status_Tracker {
	uint8_t     data_type_enabled;					            // what type of data is enabled
	uint8_t     sample_count_enabled;				            // does me11 provide sample count
	uint32_t    sample_count;
	uint8_t     data_buf_storage[512];				            // store data read from SH
	ss_data_req algo_callbacks[SH_NUM_CURRENT_ALGOS];
	ss_data_req sensor_callbacks[SH_NUM_CURRENT_SENSORS];
	uint8_t     sensor_enabled_mode[SH_NUM_CURRENT_SENSORS];
	uint8_t     algo_enabled_mode[SH_NUM_CURRENT_ALGOS];

} Max30101_SH_Status_Tracker_t;

typedef struct {
	uint32_t led1;
	uint32_t led2;
	uint32_t led3;
	uint32_t led4;
} max30101_mode1_data;

// BPT Data Struct
typedef struct {
	uint8_t  status;
	uint8_t  sys_bp;
	uint8_t  dia_bp;
	uint8_t  prog;
	uint16_t hr;
	uint16_t spo2;
	uint16_t r_value;
	uint8_t  pulse_flag;
	uint16_t ibi;
	uint8_t  spo2_conf;
	uint8_t  bpt_rpt;
	uint8_t  spo2_rpt;
	uint8_t  end_bpt;
} bpt_mode1_2_data;


/*DEMO RELATED DATA TO REPORT RESULTS FROM MAIN APPLICATION*/
uint8_t  bptMesurementProgress       = 0;

static uint8_t is_agc_usage_required = ENABLE_AGC_USAGE;

// Max30101 Default Callbacks
static void max30101_data_rx(uint8_t* data_ptr) {

	v3status.bio_red = (data_ptr[3] << 8) | data_ptr[4];
	//printLog("bio_red=%d \r\n", v3status.bio_red);
#if 0
	max30101_mode1_data sample;
	sample.led1 = (data_ptr[0] << 16) | (data_ptr[1] << 8)  | data_ptr[2];
	sample.led2 = (data_ptr[3] << 16) | (data_ptr[4] << 8)  | data_ptr[5];
	sample.led3 = (data_ptr[6] << 16) | (data_ptr[7] << 8)  | data_ptr[8];
	sample.led4 = (data_ptr[9] << 16) | (data_ptr[10] << 8) | data_ptr[11];
	printLog("led1=%d led2=%d  led3=%d  led4=%d \r\n", (int)sample.led1, (int)sample.led2, (int)sample.led3, (int)sample.led4);
#endif
}

uint16_t re_trigger = 0;
static void bpt_data_rx(uint8_t* data_ptr) {

	//See API doc for data format
    bpt_mode1_2_data sample;
	sample.status     = data_ptr[0];
	sample.prog       = data_ptr[1];
	sample.hr         = (data_ptr[2] << 8) | data_ptr[3];
	sample.sys_bp     = data_ptr[4];
	sample.dia_bp     = data_ptr[5];
	sample.spo2       = (data_ptr[6] << 8) | data_ptr[7];
	sample.r_value    = (data_ptr[8] << 8) | data_ptr[9];
	sample.pulse_flag = data_ptr[10];
	sample.ibi        = (data_ptr[11] << 8) | data_ptr[12];
	sample.spo2_conf  = data_ptr[13];
	sample.bpt_rpt    = data_ptr[14];
	sample.spo2_rpt   = data_ptr[15];
	sample.end_bpt    = data_ptr[16];
#if 0
	uint16_t spo21 = sample.spo2/10;
	uint16_t spo22 = sample.spo2 - spo21 * 10;
	if(appState  == ST_EXAMPLEUSER_ESTIMATION_MEASUREMENT)
	{
		printLog("E%3d: status=%d(%d) prog=%d hr=%d sys=%d dia=%d spo2=%d.%d\r\n",
				//"spo2_conf= %d ibi=%d pulse_flag=%d bpt_rpt=%d spo2_rpt%d end_bpt=%d\r\n"
				calibrationTimer_read(),
				sample.status,
				re_trigger,
				sample.prog,
				sample.hr/10,
				sample.sys_bp,
				sample.dia_bp,
				spo21,
				spo22
				//,sample.spo2_conf, sample.ibi, sample.pulse_flag, sample.bpt_rpt, sample.spo2_rpt, sample.end_bpt
				);
	}
	else
	{
		printLog("C%3d: status=%d(%d) prog=%d hr=%d sys=%d dia=%d spo2=%d.%d\r\n",
				//"spo2_conf= %d ibi=%d pulse_flag=%d bpt_rpt=%d spo2_rpt%d end_bpt=%d\r\n"
				calibrationTimer_read(),
				sample.status,
				re_trigger,
				sample.prog,
				sample.hr/10,
				sample.sys_bp,
				sample.dia_bp,
			    spo21,
				spo22
				//,sample.spo2_conf, sample.ibi, sample.pulse_flag, sample.bpt_rpt, sample.spo2_rpt, sample.end_bpt
				);
	}
#elif 0
	uint16_t spo21 = sample.spo2/10;
	uint16_t spo22 = sample.spo2 - spo21 * 10;
	if(appState  == ST_EXAMPLEUSER_ESTIMATION_MEASUREMENT)
	{
		printLog("E%3d: status=%d prog=%d hr=%d sys=%d dia=%d spo2=%d.%d r=, %d, ibi=, %d\r\n",
				calibrationTimer_read(),
				sample.status,
				sample.prog,
				sample.hr/10,
				sample.sys_bp,
				sample.dia_bp,
				spo21,
				spo22,
				sample.r_value,
				sample.ibi
				);
	}
	else
	{
		printLog("C%3d: status=%d prog=%d hr=%d sys=%d dia=%d spo2=%d.%d r=, %d, ibi=,%d \r\n",
				calibrationTimer_read(),
				sample.status,
				sample.prog,
				sample.hr/10,
				sample.sys_bp,
				sample.dia_bp,
			    spo21,
				spo22,
				sample.r_value,
				sample.ibi
				);
	}
#endif
	bptMesurementProgress    = sample.prog;
#if 0
  //if((sample.status == 1)&&(sample.prog == 100))
	if(sample.prog == 100)
	{
		if((sample.hr != 0)&&(sample.sys_bp != 0)&&(sample.dia_bp != 0)&&(sample.spo2 != 0))
		if(appState  == ST_EXAMPLEUSER_ESTIMATION_MEASUREMENT)
		{
			re_trigger = 100;
		}
	}

	//if(re_trigger >= 100)
	{
	  //if((sample.hr == 0)&&(sample.sys_bp == 0)&&(sample.dia_bp == 0)&&(sample.spo2 == 0))
	    if((sample.sys_bp == 0)&&(sample.dia_bp == 0))
		{
			if((sample.prog == 100)||(re_trigger >= 100))
			{
				re_trigger++;
				if(re_trigger > 500)
				{
				  re_trigger = 0;
				  appState  = ST_EXAMPLEUSER_ESTIMATION_RE_MEASUREMENT;//ST_EXAMPLEUSER_FAILURE;
				}
			}
		}
	}

    if(re_trigger >= 100)
    {
        if((sample.sys_bp != 0)&&(sample.dia_bp != 0))
        {
            if((sample.hr == 0)||(sample.spo2 == 0))
            {
              re_trigger = 0;
              appState  = ST_EXAMPLEUSER_ESTIMATION_RE_MEASUREMENT;//ST_EXAMPLEUSER_FAILURE;
            }
        }
    }
#endif
  //fill in v3status message
    v3status.bio_status = sample.status;
    v3status.bio_sys_bp = sample.sys_bp;
    v3status.bio_dia_bp = sample.dia_bp;
    v3status.bio_prog   = sample.prog;
    v3status.bio_hr     = sample.hr;
    v3status.bio_spo2   = sample.spo2;
    v3status.bio_state  = (uint8_t) appState;
}

static void agc_data_rx(uint8_t* data_ptr) {
	//NOP: AGC does not collect data
}
// end of Max30101 Default Callbacks

static Max30101_SH_Status_Tracker_t * get_config_struct() {
	static Max30101_SH_Status_Tracker_t glbl_max3010x_status_track;
	return &glbl_max3010x_status_track;
}

static void initialize_config_struct() {
	Max30101_SH_Status_Tracker_t *p_glbl_max3010x_status_track = get_config_struct();

		//set all the values to 0
	memset(p_glbl_max3010x_status_track, 0, sizeof(*p_glbl_max3010x_status_track));
	// max30101
	p_glbl_max3010x_status_track->sensor_callbacks[SH_SENSORIDX_MAX30101].data_size = SSMAX30101_MODE1_DATASIZE;
	p_glbl_max3010x_status_track->sensor_callbacks[SH_SENSORIDX_MAX30101].rx_data_parser = &max30101_data_rx;
	// agc
	p_glbl_max3010x_status_track->algo_callbacks[SH_ALGOIDX_AGC].data_size = SSAGC_MODE1_DATASIZE;
	p_glbl_max3010x_status_track->algo_callbacks[SH_ALGOIDX_AGC].rx_data_parser = &agc_data_rx;
	// whrm
	p_glbl_max3010x_status_track->algo_callbacks[SH_ALGOIDX_BPT].data_size = SSBPT_MODE1_2_DATASIZE;
	p_glbl_max3010x_status_track->algo_callbacks[SH_ALGOIDX_BPT].rx_data_parser = &bpt_data_rx;


}

void SH_Max3010x_data_report_execute(void) {
	int num_samples, databufLen;
	uint8_t *databuf;

	Max30101_SH_Status_Tracker_t *p_glbl_max3010x_status_track = get_config_struct();

	// prepare the buffer to store the results
	databuf = p_glbl_max3010x_status_track->data_buf_storage;
	databufLen = sizeof(p_glbl_max3010x_status_track->data_buf_storage);

	// poll SH
	sh_ss_execute_once(databuf, databufLen, &num_samples);

	if(num_samples) {
		//Skip status byte
		uint8_t *data_ptr = &databuf[1];

		int i = 0;
		for (i = 0; i < num_samples; i++) {
			int sh_data_type = p_glbl_max3010x_status_track->data_type_enabled;
			if (p_glbl_max3010x_status_track->sample_count_enabled) {
				p_glbl_max3010x_status_track->sample_count = *data_ptr++;
			}
			//Chop up data and send to modules with enabled sensors
			if (sh_data_type == SS_DATATYPE_RAW || sh_data_type == SS_DATATYPE_BOTH) {
				for (int i = 0; i < SH_NUM_CURRENT_SENSORS; i++) {
					if (p_glbl_max3010x_status_track->sensor_enabled_mode[i]) {
						p_glbl_max3010x_status_track->sensor_callbacks[i].rx_data_parser(data_ptr);
						data_ptr += p_glbl_max3010x_status_track->sensor_callbacks[i].data_size;
					}
				}
			}
			if (sh_data_type == SS_DATATYPE_ALGO || sh_data_type == SS_DATATYPE_BOTH) {
				for (int i = 0; i < SH_NUM_CURRENT_ALGOS; i++) {
					if (p_glbl_max3010x_status_track->algo_enabled_mode[i]) {
						p_glbl_max3010x_status_track->algo_callbacks[i].rx_data_parser(data_ptr);
						data_ptr += p_glbl_max3010x_status_track->algo_callbacks[i].data_size;
					}
				}
			}
		}
	}
}

static uint8_t read_state = 0;
static int num_samples = 0;
static int num_samplesLeft = 0;
static int databufLen;
static uint8_t *databuf;
static Max30101_SH_Status_Tracker_t *p_glbl_max3010x_status_track;
#define READ_SAMPLES_LIMIT  8
void SH_Max3010x_data_report_execute02(void)
{
	static int status = -1;

	switch(read_state)
	{
	   case 0:
	   {
		   if(m_irq_received == false)
		   {
			   num_samples = 0;
			   return;
		   }
		   p_glbl_max3010x_status_track = get_config_struct();

		   // prepare the buffer to store the results
		   databuf = p_glbl_max3010x_status_track->data_buf_storage;
		   databufLen = sizeof(p_glbl_max3010x_status_track->data_buf_storage);

		   sh_disable_irq_mfioevent();
		   sh_clear_mfio_event_flag();

		   uint8_t hubStatus = 0;
		   status = sh_get_sensorhub_status(&hubStatus);
		   if(status != 0x00 /*SS_SUCCESS*/){
			   num_samples = 0;
			   sh_enable_irq_mfioevent();
			   return;
		   }

		   if (!(hubStatus & SS_MASK_STATUS_DATA_RDY))
		   {
			   sh_enable_irq_mfioevent();
			   return;
		   }

		   status = sh_num_avail_samples(&num_samples);
		   if (status != 0x00 /*SS_SUCCESS*/){
			   num_samples = 0;
			   sh_enable_irq_mfioevent();
			   read_state = 0;
			   return;
		   }

		   int sample_size;
		   fifo_sample_size(data_type, &sample_size);
			   /*DEBUG *///
		   int bytes_to_read = num_samples * sample_size + 1; //+1 for status byte
		   if ( bytes_to_read > databufLen) {
			   //Reduce number of samples to read to fit in buffer
			   num_samples = (databufLen - 1) / sample_size;
		   }

		   if(num_samples > READ_SAMPLES_LIMIT)
		   {
			   num_samplesLeft = num_samples - READ_SAMPLES_LIMIT;
			   num_samples = READ_SAMPLES_LIMIT;
		   }
		   else
			   num_samplesLeft = 0;

		   //wait_ms(5);
		   status = sh_read_fifo_data(num_samples, sample_size, &databuf[0], databufLen);
		   if(status != 0x00 /*SS_SUCCESS*/){
			   num_samples = 0;
			   sh_enable_irq_mfioevent();
			   read_state = 0;
			   return;
		   }
		   sh_enable_irq_mfioevent();

		   if(num_samples)
		   {
			   //Skip status byte
			   uint8_t *data_ptr = &databuf[1];

			   int i = 0;
			   for (i = 0; i < num_samples; i++) {
				   int sh_data_type = p_glbl_max3010x_status_track->data_type_enabled;
				   if (p_glbl_max3010x_status_track->sample_count_enabled) {
					   p_glbl_max3010x_status_track->sample_count = *data_ptr++;
				   }
				   //Chop up data and send to modules with enabled sensors
				   if (sh_data_type == SS_DATATYPE_RAW || sh_data_type == SS_DATATYPE_BOTH) {
					   for (int i = 0; i < SH_NUM_CURRENT_SENSORS; i++) {
						   if (p_glbl_max3010x_status_track->sensor_enabled_mode[i]) {
							   p_glbl_max3010x_status_track->sensor_callbacks[i].rx_data_parser(data_ptr);
							   data_ptr += p_glbl_max3010x_status_track->sensor_callbacks[i].data_size;
						   }
					   }
				   }
				   if (sh_data_type == SS_DATATYPE_ALGO || sh_data_type == SS_DATATYPE_BOTH) {
					   for (int i = 0; i < SH_NUM_CURRENT_ALGOS; i++) {
						   if (p_glbl_max3010x_status_track->algo_enabled_mode[i]) {
							   p_glbl_max3010x_status_track->algo_callbacks[i].rx_data_parser(data_ptr);
							   data_ptr += p_glbl_max3010x_status_track->algo_callbacks[i].data_size;
						   }
					   }
				   }
			   }
		   }
		   if(num_samplesLeft)
		   {
			   num_samples = num_samplesLeft;
			   read_state = 1;
			   break;
		   }
		   else
		   {
			   read_state = 0;
			   break;
		   }
	   }
	   break;

	   case 1:
	   {
		   int sample_size;
		   fifo_sample_size(data_type, &sample_size);
			   /*DEBUG *///
		   int bytes_to_read = num_samples * sample_size + 1; //+1 for status byte
		   if ( bytes_to_read > databufLen) {
			   //Reduce number of samples to read to fit in buffer
			   num_samples = (databufLen - 1) / sample_size;
		   }

		   if(num_samples > READ_SAMPLES_LIMIT)
		   {
			   num_samplesLeft = num_samples - READ_SAMPLES_LIMIT;
			   num_samples = READ_SAMPLES_LIMIT;
		   }
		   else
			   num_samplesLeft = 0;

		   //wait_ms(5);
		   status = sh_read_fifo_data(num_samples, sample_size, &databuf[0], databufLen);
		   if(status != 0x00 /*SS_SUCCESS*/){
			   num_samples = 0;
			   sh_enable_irq_mfioevent();
			   read_state = 0;
			   return;
		   }
		   sh_enable_irq_mfioevent();

		   if(num_samples)
		   {
			   //Skip status byte
			   uint8_t *data_ptr = &databuf[1];

			   int i = 0;
			   for (i = 0; i < num_samples; i++) {
				   int sh_data_type = p_glbl_max3010x_status_track->data_type_enabled;
				   if (p_glbl_max3010x_status_track->sample_count_enabled) {
					   p_glbl_max3010x_status_track->sample_count = *data_ptr++;
				   }
				   //Chop up data and send to modules with enabled sensors
				   if (sh_data_type == SS_DATATYPE_RAW || sh_data_type == SS_DATATYPE_BOTH) {
					   for (int i = 0; i < SH_NUM_CURRENT_SENSORS; i++) {
						   if (p_glbl_max3010x_status_track->sensor_enabled_mode[i]) {
							   p_glbl_max3010x_status_track->sensor_callbacks[i].rx_data_parser(data_ptr);
							   data_ptr += p_glbl_max3010x_status_track->sensor_callbacks[i].data_size;
						   }
					   }
				   }
				   if (sh_data_type == SS_DATATYPE_ALGO || sh_data_type == SS_DATATYPE_BOTH) {
					   for (int i = 0; i < SH_NUM_CURRENT_ALGOS; i++) {
						   if (p_glbl_max3010x_status_track->algo_enabled_mode[i]) {
							   p_glbl_max3010x_status_track->algo_callbacks[i].rx_data_parser(data_ptr);
							   data_ptr += p_glbl_max3010x_status_track->algo_callbacks[i].data_size;
						   }
					   }
				   }
			   }
		   }
		   if(num_samplesLeft)
		   {
			   num_samples = num_samplesLeft;
			   break;
		   }
		   else
		   {
			   read_state = 0;
			   break;
		   }
	   }
	   break;

	   default:
	   {
		   read_state = 0;
	   }
	   break;
	}
}

/*CALIB: 1 , 0*/
/*ESTIM: 2 , 1*/
int SH_Max3010x_default_init( const int algoExecutionMode, const int agc_usage ) {

	int status;

 	// first initialize the global config struct, status tracker.
	initialize_config_struct();

	Max30101_SH_Status_Tracker_t *p_glbl_max3010x_status_track = get_config_struct();

	/* ME11 initialization based on Smart Sensor API commands*/
	status = sh_set_data_type(SS_DATATYPE_BOTH, false);
	if (status != 0) {
		printLog("\r\n err=%d\r\n", COMM_GENERAL_ERROR);
		printLog("FAILED at line %d\n", __LINE__);
		return COMM_GENERAL_ERROR;
	}else {
		p_glbl_max3010x_status_track->data_type_enabled    = SS_DATATYPE_BOTH;
		p_glbl_max3010x_status_track->sample_count_enabled = false;
	}

	status = sh_set_fifo_thresh(8);                                               //Jason, 4, 8, 15
	if (status != 0) {
		printLog("\r\n err=%d\r\n", COMM_GENERAL_ERROR);
		printLog("FAILED at line %d\n", __LINE__);
		return COMM_GENERAL_ERROR;
	}

	if( algoExecutionMode == bptExecModeEstimation) {

		if( agc_usage == ENABLE_AGC_USAGE)                       // ENABLE_AGC_USAGE = 1, Jason
		{

			status =  sh_enable_algo(SH_ALGOIDX_AGC, SSAGC_MODE1_DATASIZE);
			if (status == SS_SUCCESS)
					p_glbl_max3010x_status_track->algo_enabled_mode[SH_ALGOIDX_AGC] = 0x01;
			else {
				printLog("\r\n err=%d\r\n",  COMM_GENERAL_ERROR);
				printLog("FAILED at line %d - agc_enabled \n", __LINE__);
				//sh_clear_mfio_event_flag();
				//sh_enable_irq_mfioevent();
				return COMM_GENERAL_ERROR;
			}

	    }else {
			status = sh_disable_algo(SH_ALGOIDX_AGC);
			if (status != SS_SUCCESS) {
				printLog("\r\n err=%d\r\n",  COMM_GENERAL_ERROR);
				printLog("FAILED at line %d - agc_enabled \n", __LINE__);
				//sh_clear_mfio_event_flag();
				//sh_enable_irq_mfioevent();
				return COMM_GENERAL_ERROR;
			}else
				p_glbl_max3010x_status_track->algo_enabled_mode[SH_ALGOIDX_AGC] = 0x00;
		}
	}
#if 0
	/* Disable IRQ based Event reporting from Sensor Hub ME11*/
	sh_disable_irq_mfioevent();

	status = sh_sensor_enable(SH_SENSORIDX_MAX30101, SSMAX30101_MODE1_DATASIZE, SH_INPUT_DATA_DIRECT_SENSOR);
	if (status != SS_SUCCESS) {
		printLog("\r\n err=%d\r\n",  COMM_GENERAL_ERROR);
		printLog("FAILED at max3010x sensor init line %d\n", __LINE__);
		sh_enable_irq_mfioevent();
		return COMM_GENERAL_ERROR;
	}
	p_glbl_max3010x_status_track->sensor_enabled_mode[SH_SENSORIDX_MAX30101] = 0x01;
    // DEBUG: Check this function again!
	status = sh_enable_algo_withmode(SH_ALGOIDX_BPT, algoExecutionMode ,SSBPT_MODE1_2_DATASIZE);
	if (status != SS_SUCCESS) {
		printLog("\r\n err=%d\r\n",  COMM_GENERAL_ERROR);
		printLog("FAILED at bpt algo init line %d\n", __LINE__);
		sh_enable_irq_mfioevent();
		return COMM_GENERAL_ERROR;
	}
	p_glbl_max3010x_status_track->algo_enabled_mode[SH_ALGOIDX_BPT] = 0x01;


	/* Enable IRQ based Event reporting from Sensor Hub ME11*/
	sh_enable_irq_mfioevent();
#endif
	if(algoExecutionMode == bptExecModeCalibration)
	{
		printLog("\r\nCalication Init err = %d\r\n",  status);
	}
	else if(algoExecutionMode == bptExecModeEstimation)
	{
		printLog("\r\n Estimation Init err = %d\r\n",  status);
	}
	return COMM_SUCCESS;

}

int SH_Max3010x_algo_start( const int algoExecutionMode)                  // Added by Jason Chen
{
	int status;

	Max30101_SH_Status_Tracker_t *p_glbl_max3010x_status_track = get_config_struct();
	/* Disable IRQ based Event reporting from Sensor Hub ME11*/
    sh_disable_irq_mfioevent();

    status = sh_sensor_enable(SH_SENSORIDX_MAX30101, SSMAX30101_MODE1_DATASIZE, SH_INPUT_DATA_DIRECT_SENSOR);
    if (status != SS_SUCCESS) {
    	printLog("\r\n err=%d\r\n",  COMM_GENERAL_ERROR);
    	printLog("FAILED at max3010x sensor init line %d\n", __LINE__);
    	sh_enable_irq_mfioevent();
    	return COMM_GENERAL_ERROR;
    }

    p_glbl_max3010x_status_track->sensor_enabled_mode[SH_SENSORIDX_MAX30101] = 0x01;

    // DEBUG: Check this function again!
    status = sh_enable_algo_withmode(SH_ALGOIDX_BPT, algoExecutionMode ,SSBPT_MODE1_2_DATASIZE);
    if (status != SS_SUCCESS) {
    	printLog("\r\n err=%d\r\n",  COMM_GENERAL_ERROR);
    	printLog("FAILED at bpt algo init line %d\n", __LINE__);
    	sh_enable_irq_mfioevent();
    	return COMM_GENERAL_ERROR;
    }
    p_glbl_max3010x_status_track->algo_enabled_mode[SH_ALGOIDX_BPT] = 0x01;

    /* Enable IRQ based Event reporting from Sensor Hub ME11*/
    sh_enable_irq_mfioevent();

    return COMM_SUCCESS;
}

int SH_Max3010x_sensor_enable(void)                                                     // Added by Jason Chen,2021.03.04
{
	int status;

	//Max30101_SH_Status_Tracker_t *p_glbl_max3010x_status_track = get_config_struct();
	/* Disable IRQ based Event reporting from Sensor Hub ME11*/
    sh_disable_irq_mfioevent();

    status = sh_sensor_enable02(SH_SENSORIDX_MAX30101, SSMAX30101_MODE1_DATASIZE, SH_INPUT_DATA_DIRECT_SENSOR);
    if (status != SS_SUCCESS) {
    	sh_enable_irq_mfioevent();
    	return COMM_GENERAL_ERROR;
    }
#if 0
    wait_ms(1 * 40);

    status = sh_sensor_enable02_status(SH_SENSORIDX_MAX30101, SSMAX30101_MODE1_DATASIZE, SH_INPUT_DATA_DIRECT_SENSOR);
    if (status != SS_SUCCESS) {
    	sh_enable_irq_mfioevent();
    	return COMM_GENERAL_ERROR;
    }

    p_glbl_max3010x_status_track->sensor_enabled_mode[SH_SENSORIDX_MAX30101] = 0x01;
#endif
    /* Enable IRQ based Event reporting from Sensor Hub ME11*/
    sh_enable_irq_mfioevent();

    return COMM_SUCCESS;
}

int SH_Max3010x_sensor_enable_status(void)                                           // Added by Jason Chen,2021.03.04
{
	int status;

	Max30101_SH_Status_Tracker_t *p_glbl_max3010x_status_track = get_config_struct();
	/* Disable IRQ based Event reporting from Sensor Hub ME11*/
    sh_disable_irq_mfioevent();

    status = sh_sensor_enable02_status(SH_SENSORIDX_MAX30101, SSMAX30101_MODE1_DATASIZE, SH_INPUT_DATA_DIRECT_SENSOR);
    if (status != SS_SUCCESS) {
    	sh_enable_irq_mfioevent();
    	return COMM_GENERAL_ERROR;
    }

    p_glbl_max3010x_status_track->sensor_enabled_mode[SH_SENSORIDX_MAX30101] = 0x01;
    /* Enable IRQ based Event reporting from Sensor Hub ME11*/
    sh_enable_irq_mfioevent();

    return COMM_SUCCESS;
}

int SH_Max3010x_algo_start_02( const int algoExecutionMode)                  // Added by Jason Chen, 2021.03.04
{
	int status;

	Max30101_SH_Status_Tracker_t *p_glbl_max3010x_status_track = get_config_struct();
	/* Disable IRQ based Event reporting from Sensor Hub ME11*/
    sh_disable_irq_mfioevent();

    // DEBUG: Check this function again!
    status = sh_enable_algo_withmode02(SH_ALGOIDX_BPT, algoExecutionMode ,SSBPT_MODE1_2_DATASIZE);
    if (status != SS_SUCCESS) {
    	printLog("\r\n err=%d\r\n",  COMM_GENERAL_ERROR);
    	printLog("FAILED at bpt algo init line %d\n", __LINE__);
    	sh_enable_irq_mfioevent();
    	return COMM_GENERAL_ERROR;
    }

    p_glbl_max3010x_status_track->algo_enabled_mode[SH_ALGOIDX_BPT] = 0x01;

    /* Enable IRQ based Event reporting from Sensor Hub ME11*/
    sh_enable_irq_mfioevent();

    return COMM_SUCCESS;
}

int SH_Max3010x_algo_start_02_status( const int algoExecutionMode)                  // Added by Jason Chen, 2021.03.04
{
	int status;

	/* Disable IRQ based Event reporting from Sensor Hub ME11*/
    sh_disable_irq_mfioevent();

    status = sh_enable_algo_withmode_status02(SH_ALGOIDX_BPT, algoExecutionMode ,SSBPT_MODE1_2_DATASIZE);
    if (status != SS_SUCCESS) {
    	printLog("\r\n err=%d\r\n",  COMM_GENERAL_ERROR);
    	printLog("FAILED at bpt algo init line %d\n", __LINE__);
    	sh_enable_irq_mfioevent();
    	return COMM_GENERAL_ERROR;
    }

    /* Enable IRQ based Event reporting from Sensor Hub ME11*/
    sh_enable_irq_mfioevent();

    return COMM_SUCCESS;
}

/* COMMAND TABLE FUNCTIONS*/
int SH_Max3010x_get_bpt_dataformat(const char *null_arg){

	printLog("\r\n  format={status,4},{irCnt,19},{hr,9},"
	 							"{prog,9},{sys_bp,9},{dia_bp,9},{spo2,8} err=0\r\n" );

	 return 0;
}

int SH_Max3010x_set_ppgreg(const char *addr_value_args){

	//CMD: set_reg ppgsensor 0xAA 0xAA
	int addr;
    int status = -1;//Jason
    int val = 0;//Jason
	if( sscanf(addr_value_args,"%*s %*s %4x %10x", &addr , &addr ) == 2 ){
        status = sh_set_reg(SH_SENSORIDX_MAX30101, (uint8_t) addr, (uint32_t) val, SSMAX30101_REG_SIZE);
        if(status == 0)
        	printLog("OK \r\n");
	}else
		printLog("ERR \r\n");

    return status;
}

int SH_Max3010x_get_ppgreg(const char *addr_arg){

	//CMD: get_reg ppgsensor 0xAA
	int addr;
	int status = -1;
	uint32_t val;

	if( sscanf(addr_arg,"%*s %*s %4x", &addr) == 1 ){
		int status = sh_get_reg(SH_SENSORIDX_MAX30101, (uint8_t) addr, &val);
        if(status == 0)
        	printLog("reg_val=%02X \r\n",(unsigned int)val);//Jason
        else
        	printLog("COMM ERR \r\n");
	}else
		printLog("ERR \r\n");

    return status;
}


int SH_Max3010x_measure_bpt_calibration(const char *null_arg){

	 int status;
	 SH_Max3010x_stop(0);
	 status = SH_Max3010x_default_init(bptExecModeCalibration , ENABLE_AGC_USAGE);
	 if(status == 0)
		 printLog("whrm started \r\n");
	 else
		 printLog("ERR");

    return status;

}

int SH_Max3010x_measure_bpt_estimation(const char *null_arg){

	 int status;

	 //SH_Max3010x_stop(0);
	 status = SH_Max3010x_default_init(bptExecModeEstimation , ENABLE_AGC_USAGE); /* ?? AGC USAGE DEFAULT ENABLE !!!!!!!*/
	 if(status == 0)
		 printLog("whrm started \r\n");
	 else
		 printLog("ERR");

   return status;

}

/*IMPORTANT: AGC usage setting should be set before BPT algorithm initialized */
int SH_Max3010x_set_ppg_agcusage(const char *config_arg){

	int status = -1;
	uint32_t val;
    if( sscanf(config_arg, "%*s %*s %*s %d", (int*)&val) == 1 ){  //Jason
    	is_agc_usage_required = (val == 0)? DISABLE_AGC_USAGE:ENABLE_AGC_USAGE;
    	status =  SS_SUCCESS;
    }

    return status;  // if command error return -1 if operational error return >0 error

}


int SH_Max3010x_set_bpt_med_config(const char *config_arg){

	int status = -1;
	uint32_t val;
	if( sscanf(config_arg, "%*s %*s %*s %d", (int*)&val) == 1 ){  //Jason
		uint8_t Temp[1] = { (uint8_t) (val & 0xFF)};
		status = sh_set_algo_cfg_extendedwait(SH_ALGOIDX_BPT, SS_CFGIDX_BP_USE_MED , &Temp[0], sizeof(Temp), 30);
	}

	//SERIALOUT("\r\n%s err=%d\r\n", CMDTABLE3010xBPT[6].cmdStr , status);
	printLog("\r\n%s err=%d\r\n", config_arg , status);
	return status;

}

int SH_Max3010x_set_bpt_sys_dia_config(const char *config_arg){

	int status = -1;
	uint32_t val[3];
	if( sscanf(config_arg, "%*s %*s %*s %u %u %u", (unsigned int *)&val[0], (unsigned int *)&val[1] , (unsigned int *)&val[2] ) == 3 ){//Jason
		uint8_t Temp[3] = { (uint8_t) (val[0] & 0xFF) , (uint8_t) (val[1] & 0xFF) , (uint8_t) (val[2] & 0xFF) };

		status = sh_set_algo_cfg_extendedwait(SH_ALGOIDX_BPT, SS_CFGIDX_BP_SYS_DIA , &Temp[0], sizeof(Temp), 30);
	}

	//SERIALOUT("\r\n%s err=%d\r\n", CMDTABLE3010xBPT[18].cmdStr , status);
	printLog("\r\n%s err=%d\r\n", config_arg , status);
	return status;

}


int SH_Max3010x_set_bpt_sysbp_config(const char *config_arg){

	int status = -1;
	uint32_t val[3];
	if( sscanf(config_arg, "%*s %*s %*s %d %d %d", (int*)&val[0], (int*)&val[1] , (int*)&val[2] ) == 3 ){   //Jason
		uint8_t Temp[3] = { (uint8_t) (val[0] & 0xFF) , (uint8_t) (val[1] & 0xFF) , (uint8_t) (val[2] & 0xFF) };

		status = sh_set_algo_cfg_extendedwait(SH_ALGOIDX_BPT, SS_CFGIDX_BP_SYS_BP_CAL , &Temp[0], sizeof(Temp), 30);
	}

	//SERIALOUT("\r\n%s err=%d\r\n", CMDTABLE3010xBPT[7].cmdStr , status);
	printLog("\r\n%s err=%d\r\n", config_arg , status);
	return status;

}

int SH_Max3010x_set_bpt_diabp_config(const char *config_arg){

	int status = -1;
	uint32_t val[3];
	if( sscanf(config_arg, "%*s %*s %*s %d %d %d", (int*)&val[0], (int*)&val[1] , (int*)&val[2] ) == 3 ){   //Jason

		uint8_t Temp[3] = { (uint8_t) (val[0] & 0xFF) , (uint8_t) (val[1] & 0xFF) , (uint8_t) (val[2] & 0xFF) };
		status = sh_set_algo_cfg_extendedwait(SH_ALGOIDX_BPT, SS_CFGIDX_BP_DIA_BP_CAL , &Temp[0], sizeof(Temp), 30);

	}

	//SERIALOUT("\r\n%s err=%d\r\n", CMDTABLE3010xBPT[8].cmdStr , status);
	printLog("\r\n%s err=%d\r\n", config_arg , status);
	return status;

}

int SH_Max3010x_get_bpt_cal_index(const char *null_arg){

	int status = -1;
	uint8_t  rxBuff[1*1 + 1];
	uint32_t val[1];
	status = sh_get_algo_cfg(SH_ALGOIDX_BPT, SS_CFGIDX_BP_CAL_INDEX, &rxBuff[0], sizeof(rxBuff));
    if( status == SS_SUCCESS) {

    	val[0] =  rxBuff[0];
    	printLog(" \r\n cal_index: %u \r\n" , (unsigned int)val[0] );//Jason
    }

    printLog("\r\n%s err=%d\r\n", CMDTABLE3010xBPT[19].cmdStr , status);
    return status;
}

int SH_Max3010x_set_bpt_cal_index(const char *config_arg){

	int status = -1;
	uint32_t val[1];
	if( sscanf(config_arg, "%*s %*s %*s %u", (unsigned int*)&val[0] ) == 1 ){
		uint8_t Temp[1] = { (uint8_t) (val[0] & 0xFF) };

		status = sh_set_algo_cfg_extendedwait(SH_ALGOIDX_BPT, SS_CFGIDX_BP_CAL_INDEX , &Temp[0], sizeof(Temp), 30);
	}

	//SERIALOUT("\r\n%s err=%d\r\n", CMDTABLE3010xBPT[20].cmdStr , status);
	printLog("\r\n%s err=%d\r\n", config_arg , status);
	return status;
}

int SH_Max3010x_get_bpt_calibration(const char *null_arg){

    int status = -1;
    uint8_t cal_result[CAL_DATA_SIZE_BYTES + STATUS_OFFSET_BYTE];

 	status = sh_get_algo_cfg(SH_ALGOIDX_BPT, SS_CFGIDX_BP_CAL_DATA, &cal_result[0], sizeof(cal_result));
    if( status == SS_SUCCESS){

    	printLog("\r\n VALUES of CALIBRATION= ");
    	unsigned int idx = 1; /*skip status byte*/
    	while( idx < sizeof(cal_result) ){
    		printLog("%02X", cal_result[idx+1]);                                               // %2X ??
    		idx++;
    	}

    }

    printLog("\r\n%s err=%d\r\n", CMDTABLE3010xBPT[9].cmdStr , status);
    return status;

}



int parse_cal_str(const char *ptr_ch, const char *cmd, uint8_t *cal_data, int cal_data_sz)
{
	char ascii_byte[] = { 0, 0, 0 };
	const char* sptr = ptr_ch + strlen(cmd);
	int found = 0;
	int ssfound;
	unsigned int val32;

	//Eat spaces after cmd
	while (*sptr == ' ') { sptr++; }
	if (*sptr == '\0')
		return -1;
	//sptr++;

	while (found < cal_data_sz) {
		if (*sptr == '\0')
			break;
		ascii_byte[0] = *sptr++;
		ascii_byte[1] = *sptr++;
		ssfound = sscanf(ascii_byte, "%x", &val32);
		if (ssfound != 1)
			break;
		*(cal_data + found) = (uint8_t)val32;
		//pr_err("cal_data[%d]=%d\r\n", found, val32);
		found++;
	}

	//SERIALOUT("total found: %d  >>>>>>\r\n", found);
	if (found < cal_data_sz)
		return -1;
	return 0;
}

int ssx_set_algo_cfg(int algo_idx, int cfg_idx, uint8_t *cfg, int cfg_sz)
{

	uint8_t cmd_bytes[] = { SS_FAM_W_ALGOCONFIG, (uint8_t)algo_idx, (uint8_t)cfg_idx };
	int status = sh_write_cmd_with_data(&cmd_bytes[0], sizeof(cmd_bytes),
								 cfg, cfg_sz,30);

	return status;
}

int SH_Max3010x_set_bpt_calibration(const char *config_arg){

/*	int status = -1;
	uint8_t cal_data[CAL_DATA_SIZE_BYTES] = {0};
    int ret = parse_cal_str( config_arg, CMDTABLE3010xBPT[10].cmdStr, cal_data, sizeof(cal_data));
	if (ret) {
		SERIALOUT("\r\n%s err=%d\r\n", CMDTABLE3010xBPT[10].cmdStr, COMM_INVALID_PARAM);
		return -1;
	}
	status = ssx_set_algo_cfg(SH_ALGOIDX_BPT, SS_CFGIDX_BP_CAL_DATA, &cal_data[0], sizeof(cal_data));

	SERIALOUT("\r\n%s err=%d\r\n", CMDTABLE3010xBPT[10].cmdStr , status);
	return status;
*/
	uint8_t cal_data[CAL_DATA_SIZE_BYTES];
	int ret = parse_cal_str(config_arg,"set_cfg bpt cal_result", cal_data, sizeof(cal_data));
	if (ret) {
		printLog("___DEBUG CAL STR PARSE ERROR \r\n");
		printLog("\r\n%s err=%d\r\n", CMDTABLE3010xBPT[10].cmdStr, COMM_INVALID_PARAM);
		return -1;
	}
	int status = ssx_set_algo_cfg(SH_ALGOIDX_BPT, SS_CFGIDX_BP_CAL_DATA, &cal_data[0], sizeof(cal_data));
	printLog("\r\n%s err=%d\r\n", CMDTABLE3010xBPT[10].cmdStr , status);
	return status;

}

int SH_Max3010x_set_bpt_date_time(const char *config_arg){

    int status = -1;//Jason
    uint32_t date_time[2];
    if( sscanf(config_arg, "%*s %*s %*s %d %d", (int*)&date_time[0], (int*)&date_time[1]) == 2 ){  //Jason
    	status = sh_set_algo_cfg_extendedwait(SH_ALGOIDX_BPT, SS_CFGIDX_BP_EST_DATE, (uint8_t*)date_time, sizeof(date_time), 30);
    }

	//SERIALOUT("\r\n%s err=%d\r\n", CMDTABLE3010xBPT[11].cmdStr , status);
	printLog("\r\n%s err=%d\r\n", config_arg , status);
	return status;

}

int SH_Max3010x_set_bpt_nonrest_config(const char *config_arg){

	int status = -1;
	uint32_t val;
    if( sscanf(config_arg, "%*s %*s %*s %d", (int*)&val) == 1 )
    {

    	uint8_t Temp[1] = { (uint8_t) (val & 0xFF)};
		status = sh_set_algo_cfg_extendedwait(SH_ALGOIDX_BPT, SS_CFGIDX_BP_EST_NONREST , &Temp[0], sizeof(Temp) , 30);
    }

    //SERIALOUT("\r\n%s err=%d\r\n", CMDTABLE3010xBPT[12].cmdStr , status);
    printLog("\r\n%s err=%d\r\n", config_arg , status);
    return status;  // if command error return -1 if operational error return >0 error

}


static int parse_cmd_data(const char* str, const char* cmd, uint32_t *vals, int vals_sz, bool hex)
{
	const char* sptr = str + strlen(cmd);
	int found = 0;
	int ssfound;

	while (found < vals_sz) {
		while (*sptr != ' ' && *sptr != '\0') { sptr++; }
		if (*sptr == '\0')
			break;
		sptr++;

		if (hex)
			ssfound = sscanf(sptr, "%x", (unsigned int*)(vals + found));//Jason
		else
			ssfound = sscanf(sptr, "%d", (unsigned int*)(vals + found));//Jason
		if (ssfound != 1)
			break;
		found++;
	}

	return found;
}


int SH_Max3010x_set_bpt_spo2_coeffs(const char *config_arg){


#if 0

	int status = -1;
	uint32_t coefs[3];
	if( sscanf(config_arg, "%*s %*s %*s %10x %10x %10x", &coefs[0], &coefs[1] , &coefs[2] ) == 3 ){
		/*need as 32bit adress is casted to uint8_t array at following sh_ser_algo_cfg function call.*/
		coefs[0] =  __builtin_bswap32 (coefs[0]);
		coefs[1] =  __builtin_bswap32 (coefs[1]);
		coefs[2] =  __builtin_bswap32 (coefs[2]);
        status = sh_set_algo_cfg_extendedwait(SH_ALGOIDX_BPT, SS_CFGIDX_BP_SPO2_COEFS, (uint8_t*)coefs, sizeof(coefs) , 30);
	}

	//SERIALOUT("\r\n%s err=%d\r\n", CMDTABLE3010xBPT[13].cmdStr , status);
	SERIALOUT("\r\n%s err=%d\r\n", config_arg , status);
	return status;  // if command error return -1 if operational error return >0 error
#else

	int status;
    uint32_t coefs[3];
    if(3 != parse_cmd_data(config_arg, "set_cfg bpt spo2_coefs", coefs, 3, true)){
    	printLog("\r\n%s err=%d\r\n", config_arg, COMM_INVALID_PARAM);
        return -1;
    }

/*    SERIALOUT(" \r\n_______DEBUG SPO2 COEFFS: ");//__DEBUG
    int i = 0;
    for( i = 0 ; i< 3 ; i++)
    	SERIALOUT("%x ", coefs[i]);
    SERIALOUT(" \r\n"); */

    status = ssx_set_algo_cfg(SH_ALGOIDX_BPT, SS_CFGIDX_BP_SPO2_COEFS, (uint8_t*)coefs, sizeof(coefs));
	if (status == SS_SUCCESS)
		printLog("\r\n%s err=%d\r\n", config_arg, COMM_SUCCESS);
	else
		printLog("\r\n%s err=%d\r\n", config_arg, COMM_GENERAL_ERROR);

	return status;

#endif

}

int SH_Max3010x_get_bpt_spo2_coeffs(const char *null_arg){

	int status = -1;
	uint8_t  rxBuff[3*4 + 1];
	uint32_t val[3];
	status = sh_get_algo_cfg(SH_ALGOIDX_BPT, SS_CFGIDX_BP_SPO2_COEFS, &rxBuff[0], sizeof(rxBuff));
    if( status == SS_SUCCESS) {

    	val[0] =  (rxBuff[1]<< 24) +  (rxBuff[2]<< 16) + (rxBuff[3]<< 8) + (rxBuff[4]<< 0);
		val[1] =  (rxBuff[5]<< 24) +  (rxBuff[6]<< 16) + (rxBuff[7]<< 8) + (rxBuff[8]<< 0);
		val[2] =  (rxBuff[9]<< 24) +  (rxBuff[10]<< 16) + (rxBuff[11]<< 8) + (rxBuff[12]<< 0);
		printLog(" \r\n spo2_calib: A=0x%x B=0x%x C=0x%x \r\n" , (unsigned int)val[0], (unsigned int)val[1], (unsigned int)val[2] );
    }

    printLog("\r\n%s err=%d\r\n", CMDTABLE3010xBPT[14].cmdStr , status);
    return status;

}

int SH_Max3010x_stop(const char *null_arg) {

	sh_disable_irq_mfioevent();
	Max30101_SH_Status_Tracker_t *p_glbl_max3010x_status_track = get_config_struct();

	for(int i = 0; i < SH_NUM_CURRENT_SENSORS; ++i) {
		if(p_glbl_max3010x_status_track->sensor_enabled_mode[i]) {
			p_glbl_max3010x_status_track->sensor_enabled_mode[i] = 0;
			sh_sensor_disable(i);
		}

	}

	for(int i = 0; i < SH_NUM_CURRENT_ALGOS; ++i) {
		if(p_glbl_max3010x_status_track->algo_enabled_mode[i]) {
			p_glbl_max3010x_status_track->algo_enabled_mode[i] = 0;
			sh_disable_algo(i);
		}
	}

	sh_clear_mfio_event_flag();
	sh_enable_irq_mfioevent();

	return 0x00;
}

