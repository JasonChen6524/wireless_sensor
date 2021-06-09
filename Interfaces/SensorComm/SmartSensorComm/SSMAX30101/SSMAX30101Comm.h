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

#ifndef _SSMAX30101COMM_H_
#define _SSMAX30101COMM_H_

#include "SensorComm.h"
#include "SSInterface.h"
#include "queue.h"
#include "app.h"

#define INCLUDE_REDCNT_ANDROID_APK

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} accel_mode1_data;

typedef struct {
	uint8_t status;
	uint8_t sys_bp;
	uint8_t dia_bp;
	uint8_t prog;
	uint16_t hr;
	uint16_t spo2;
	uint16_t r_value;
#if defined(BPT_OLDER_THAN_40_2_7)
	uint8_t hr_excthresh;
#else
	uint8_t pulse_flag;
	uint16_t ibi;
	uint8_t spo2_conf;
#endif

} bpt_mode1_2_data;

typedef struct {
	uint32_t led1;
	uint32_t led2;
	uint32_t led3;
	uint32_t led4;
} max30101_mode1_data;

typedef struct {
	uint16_t hr;
	uint8_t hr_conf;
	uint16_t spo2;
	uint8_t status;
} whrm_mode1_data;

/* PRIVATE TYPE DEFINITIONS */
typedef enum _cmd_state_t {
	get_format_ppg_0,
	get_format_ppg_4,
	get_format_ppg_9,
	get_format_bpt_0,
	get_format_bpt_1,
	read_ppg_0,
	read_ppg_1,
	read_ppg_4,
	read_ppg_9,
	read_bpt_0,
	read_bpt_1,
	get_reg_ppg,
	set_reg_ppg,
	dump_reg_ppg,
	set_agc_dis,
	set_agc_en,
	set_cfg_bpt_med,
	set_cfg_bpt_sys_bp,
	set_cfg_bpt_dia_bp,
	get_cfg_bpt_cal_data,
	set_cfg_bpt_cal_data,
	set_cfg_bpt_date_time,
	set_cfg_bpt_nonrest,
    set_cfg_bpt_spo2_coefs,
	self_test_ppg_os24,
	self_test_ppg_acc,
	backup_shut_down,
	backup_wake_up,
	cfg_accel_sh,
	set_cfg_bpt_cal_index,                                 // Added by Jason
	set_cfg_bpt_sys_dia,                                   // Added by Jason
	NUM_CMDS_VALUE,
} cmd_state_t;

typedef struct __attribute__((packed)) {
	uint32_t start_byte	:8;

	uint32_t sample_cnt	:32;
	uint32_t led1	:20;
	uint32_t led2	:20;
	uint32_t led3	:20;
	uint32_t led4	:20;
	uint32_t x	    :14;	//Represent values of 0.000 through 8.191
	uint32_t y	    :14;	//Represent values of 0.000 through 8.191
	uint32_t z	    :14;	//Represent values of 0.000 through 8.191
	uint32_t hr	    :12;	//Represent values of 0.0 through 204.7
	uint32_t spo2	:11;	//Represent values of 0.0 through 102.3 (only need up to 100.0)
	uint32_t status	:8;

	uint8_t	:0;			//Align CRC byte on byte boundary
	uint8_t crc8:8;
} ds_pkt_data_mode1;

typedef struct __attribute__((packed)) {
		uint32_t start_byte  :8;
		uint32_t status      :6;  // MYG was 4 bit originally
		uint32_t irCnt       :19;
		uint32_t redCnt      :19;
		uint32_t hr          :9;
		uint32_t prog        :9;
		uint32_t sys_bp      :9;
		uint32_t dia_bp      :9;
        uint32_t spo2        :8;
        uint32_t hr_excthresh:8;
        uint32_t r           :16;
        uint32_t ibi         :16;
        uint32_t spo2_conf   :8;
        uint32_t dummy       :3;
		uint8_t              :0; //Align to next byte
		uint8_t  crc8        :8;
	} ds_pkt_bpt_data;


typedef struct __attribute__((packed)){
	uint8_t current_operating_mode;  // mode 1 & 2
	// WHRM data
	uint16_t hr;         	         // mode 1 & 2
	uint8_t  hr_conf;     	         // mode 1 & 2
	uint16_t rr;         	         // mode 1 & 2
	uint8_t  rr_conf;		         // mode 1 & 2
	uint8_t  activity_class;         // mode 1 & 2
	// WSPO2 data
	uint16_t r;						 // mode 1 & 2
	uint8_t  spo2_conf;		         // mode 1 & 2
	uint16_t spo2;			         // mode 1 & 2
	uint8_t  percentComplete;		 // mode 1 & 2
	uint8_t  lowSignalQualityFlag;	 // mode 1 & 2
	uint8_t  motionFlag;			 // mode 1 & 2
	uint8_t  lowPiFlag;				 // mode 1 & 2
	uint8_t  unreliableRFlag;		 // mode 1 & 2
	uint8_t  spo2State;   			 // mode 1 & 2
	uint8_t  scd_contact_state;
	//Extended Report (mode2)
	uint32_t walk_steps;	         // mode 2
	uint32_t run_steps;		         // mode 2
	uint32_t kcal;			         // mode 2
	uint32_t totalActEnergy;		 // mode 2
	uint8_t  is_led_cur1_adj;	     // mode 2
	uint16_t adj_led_cur1;	         // mode 2
	uint8_t  is_led_cur2_adj;        // mode 2
	uint16_t adj_led_cur2;	         // mode 2
	uint8_t  is_led_cur3_adj;        // mode 2
	uint16_t adj_led_cur3;	         // mode 2
	uint8_t  is_int_time_adj;	     // mode 2
	uint8_t  t_int_code;	            // mode 2
	uint8_t  is_f_smp_adj;	         // mode 2
	uint8_t  adj_f_smp;		         // mode 2
	uint8_t  smp_ave;		         // mode 2
	uint8_t  hrm_afe_state;          // mode 2
	uint8_t  is_high_motion;	     // mode 2
	uint8_t  ibi_offset;
} whrm_wspo2_suite_modeX_data;

/* TODO: Implement:  REVIEW FIELDS!!!*/
typedef struct __attribute__((packed)) {

    uint32_t start_byte			 :8;
	uint32_t sample_cnt			 :8;
	uint32_t sampleTime;

	uint32_t grnCnt				 :20;
	uint32_t grn2Cnt			 :20;
	uint32_t irCnt	    		 :20;
	uint32_t redCnt				 :20;

	uint32_t x					 :14;	//Represent values of 0.000 through 8.191
	uint32_t y					 :14;	//Represent values of 0.000 through 8.191
	uint32_t z					 :14;	//Represent values of 0.000 through 8.191

	uint32_t curr_opmode         :4;
	uint32_t hr					 :12;	//Represent values of 0.0 through 204.7
	uint32_t hr_confidence  	 :8;	//Represent values of 0.0 through 100
	uint32_t rr					 :14;	//
	uint32_t rr_confidence  	 :8;	//Represent values of 0.0 through 100
	uint32_t activity			 :4;
	uint32_t r					 :12;
	uint32_t spo2_confidence  	 :8;	//Represent values of 0.0 through 100
	uint32_t spo2				 :11;	//Represent values of 0.0 through 102.3 (only need up to 100.0)
	uint32_t percentComplete	 :8;
	uint32_t lowSignalQualityFlag:1;
	uint32_t motionFlag			 :1;
	uint32_t lowPiFlag			 :1;
	uint32_t unreliableRflag     :1;
	uint32_t spo2State			 :4;
	uint32_t scdState			 :4;
	uint32_t ibiOffset           :8;
	uint8_t						 :0;			//Align CRC byte on byte boundary
	uint8_t crc8				 :8;

} ds_pkt_data_mode1_whrm_wspo2_suite;

typedef struct __attribute__((packed)) {

    uint32_t start_byte			 :8;
    uint32_t sample_cnt			 :8;
	uint32_t sampleTime          :32;

	uint32_t grnCnt				 :20;
	uint32_t grn2Cnt			 :20;
	uint32_t irCnt	    		 :20;
	uint32_t redCnt				 :20;

	uint32_t x					 :14;	//Represent values of 0.000 through 8.191
	uint32_t y					 :14;	//Represent values of 0.000 through 8.191
	uint32_t z					 :14;	//Represent values of 0.000 through 8.191

	uint32_t curr_opmode         :4;
	uint32_t hr					 :12;	//Represent values of 0.0 through 204.7
	uint32_t hr_confidence  	 :8;	//Represent values of 0.0 through 100
	uint32_t rr					 :14;	//Represent values of 0.0 through 102.3 (only need up to 100.0)
	uint32_t rr_confidence  	 :8;	//Represent values of 0.0 through 100
	uint32_t activity			 :4;
	uint32_t r					 :12;
	uint32_t spo2_confidence  	 :8;	//Represent values of 0.0 through 100
	uint32_t spo2				 :11;	//Represent values of 0.0 through 102.3 (only need up to 100.0)
	uint32_t percentComplete	 :8;
	uint32_t lowSignalQualityFlag:1;
	uint32_t motionFlag			 :1;
	uint32_t lowPiFlag			 :1;
	uint32_t unreliableRflag     :1;
	uint32_t spo2State			 :4;
	uint32_t scdState			 :4;

	uint32_t walk_steps          :32;
	uint32_t run_steps           :32;
	uint32_t kcal                :32;
	uint32_t totalActEnergy      :32;
	uint32_t ibiOffset			 :8;
	uint8_t						 :0;			//Align CRC byte on byte boundary
	uint8_t crc8				 :8;

} ds_pkt_data_mode_maxim_sensors_app;

void SSMAX30101Comm_init(void);

// sensor and algo status
  //status_algo_sensors_st sensor_algo_en_dis_;

extern bool             cal_data_flag;
extern volatile uint8_t data_report_mode;

inline bool get_cal_data_flag(void)  {
	return cal_data_flag;
}

inline bool is_active_measurement(void)  {
	bool is_asctive_mes = ( data_report_mode == read_bpt_0 || data_report_mode == read_bpt_1 || data_report_mode == read_ppg_4 || data_report_mode == read_ppg_9 )? true:false;
	return is_asctive_mes;
}

/**
 * @brief	SSMAX30101Comm Command handler class for communication with MAX30101 on SmartSensor board
 * @details
 */
//class SSMAX30101Comm:	public SensorComm
typedef void(*set_sensorhub_accel_callback)(void);
typedef struct
{
	get_type_callback            get_type;
	is_visible_callback          is_visible;
	get_part_info_callback       get_part_info;
	//is_enabled_callback          is_enabled;
	get_part_name_callback       get_part_name;
	get_algo_ver_callback        get_algo_ver;
	stop_callback                stop;
	parse_command_callback       parse_command;
	data_report_execute_callback data_report_execute;
	set_sensorhub_accel_callback set_sensorhub_accel;

	SensorComm_Set_Ble_Status_callback SensorComm_Set_Ble_Status;
} SSMAX30101Comm;

extern SSMAX30101Comm ssMAX30101;

#endif /* _SSMAX30101COMM_H_ */
