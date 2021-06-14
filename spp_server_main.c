/***********************************************************************************************//**
 * \file   spp_server_main.c
 * \brief  SPP server example
 *
 *
 ***************************************************************************************************
 * <b> (C) Copyright 2016 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

/* Board headers */
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"


#include <stdio.h>
#include "retargetswo.h"
#include "sleep.h"
#include "spp_utils.h"
#include "em_usart.h"
#include "sl_sleeptimer.h"
#include "infrastructure.h"

/* Vecttor3 */
#include "global.h"
#include "v3.h"
#include "i2c.h"

#include "queue.h"
#include "DSInterface.h"
#include "peripherals.h"
#include "utils.h"

/***************************************************************************************************
  Local Macros and Definitions
 **************************************************************************************************/
// Moved to V3.h
//#define STATE_ADVERTISING 1
//#define STATE_CONNECTED   2
//#define STATE_SPP_MODE    3

/* Maximum number of iterations when polling UART RX data before sending data over BLE connection
 * set value to 0 to disable optimization -> minimum latency but may decrease throughput */
#define UART_POLL_TIMEOUT  5000

/* soft timer handles */
#define TIC_TIMER_HANDLE 1
#define OS_TIMER_HANDLE 2

//bool dddd = 0;
#define TIC_TIMER_CONST 32768
#define TIC_TIMER_PERSEC 10

struct queue_t BLEQUEUE;
static uint8_t BLEOutBuffer[BLE_NOTIFY_CHAR_ARR_SIZE * MAX_BLE_QUEUE];

enum
{
  //HANDLE_V3, HANDLE_EDDYSTONE, HANDLE_IBEACON  //include this code for Eddystone
  HANDLE_V3, HANDLE_IBEACON
};

//ibeacon setup call
void bcnSetupAdvBeaconing(void);

extern void bpt_main(void); // Temporarily put here, belongs elsewhere
extern void bpt_main_reset(void); // Added by Jason, 2021.01.07

#if 0 //include this code for Eddystone
// Eddystone Advertising data
#define EDDYSTONE_DATA_LEN (23)

static const uint8_t eddystone_data[EDDYSTONE_DATA_LEN] = {
  0x03,          // Length of service list
  0x03,          // Service list
  0xAA, 0xFE,    // Eddystone ID
  0x12,          // Length of service data
  0x16,          // Service data
  0xAA,  0xFE,   // Eddystone ID
  0x10,          // Frame type Eddystone-URL
  0x00,          // Tx power
  0x00,          // 0x00=http://www., 0x01=https://www.
  'a','r','t','a','f','l','e','x','.','c','o','m'
};
#endif

/***************************************************************************************************
 Local Variables
 **************************************************************************************************/
static uint8 _conn_handle = 0xFF;
static int _main_state;

tsCounters _sCounters;

static uint8 _max_packet_size = 20; // Maximum bytes per one packet
static uint8 _min_packet_size = 20; // Target minimum bytes for one packet
static uint8 boot_to_dfu = 0;

static void reset_variables()
{
	int ret;
	_conn_handle = 0xFF;
	_main_state = STATE_ADVERTISING;
   v3status.spp = STATE_ADVERTISING;
    //gecko_cmd_hardware_set_soft_timer(0, TIC_TIMER_HANDLE, 0);
    
	_max_packet_size = 20;

	memset(&_sCounters, 0, sizeof(_sCounters));
   
   v3CommBuf.rxhead = 0;
   v3CommBuf.rxtail = 0;
   v3CommBuf.txhead = 0;
   v3CommBuf.txtail = 0;

	ret = queue_init(&BLEQUEUE, BLEOutBuffer, BLE_NOTIFY_CHAR_ARR_SIZE, BLE_NOTIFY_CHAR_ARR_SIZE * MAX_BLE_QUEUE);
	if(ret != 0)
		printLog("queue_init has failed\r\n");
}


//DSInterface dsInterface(&microUSB);


int BLE_TransferMesSamplesFromQueue(void)
{
	int ret;
	uint16 result;
	uint8_t data_transfer[BLE_NOTIFY_CHAR_ARR_SIZE];

	//printLog("BLE data \r\n");
	if (BLEQUEUE.num_item >= 1)
	{
		ret = dequeue(&BLEQUEUE, data_transfer);
		if(ret < 0) return 0;
		pr_info("dequeued data for tx, %d remain\r\n", BLEQUEUE.num_item);                           // Modified by Jason
		do {
		  //result = gecko_cmd_gatt_server_send_characteristic_notification(_conn_handle, gattdb_gatt_spp_data, len, data)->result;
			result = gecko_cmd_gatt_server_send_characteristic_notification(_conn_handle, gattdb_notifyChar, BLE_NOTIFY_CHAR_ARR_SIZE, data_transfer)->result;
			_sCounters.num_writes++;
		} while(result == bg_err_out_of_memory);

		if (result != 0) {
			pr_err("Unexpected error: %x\r\n", result);
		} else {
			_sCounters.num_pack_sent++;
			_sCounters.num_bytes_sent += BLE_NOTIFY_CHAR_ARR_SIZE;
		}
	}

	return 0;
}

bool BLE_Interface_Exists(void)
{
	//return (bleState == BLE_CONNECTED);
	return (v3status.spp == STATE_SPP_MODE);
}

int BLE_AddtoQueue(uint8_t *data_transfer, int32_t buf_size, int32_t data_size, int32_t line) {

	int ret = 0;
	//printf("size is: %d\r\n", size);
	// TODO: Append a known character to the byte array in case size is
	// less than 20 bytes
	while ((data_size % BLE_NOTIFY_CHAR_ARR_SIZE) && data_size < buf_size)
		data_transfer[data_size++] = 0;
	//mxm_assert_msg(!(data_size % BLE_NOTIFY_CHAR_ARR_SIZE), "BLE packet size must be multiple of 32 bytes");

	while(data_size > 0){
		ret = enqueue(&BLEQUEUE, data_transfer);
		data_size -= BLE_NOTIFY_CHAR_ARR_SIZE;
		data_transfer += BLE_NOTIFY_CHAR_ARR_SIZE;
	}

	if(ret != 0)
		printLog("BLE_AddtoQueue has failed, ret=%d, Line:%d\r\n", ret, __LINE__);
    pr_info("BLE_AddtoQueue, line=%d\r\n", (int)line);                         // Added by Jason
	return ret;
}

void BLE_reset_queue(void){
	queue_reset(&BLEQUEUE);
    memset( BLEQUEUE.base, 0, BLEQUEUE.buffer_size);                                      // Added by Jason
}


void send_spp_msg()
{
	uint8 len = 0;
	uint8 data[256];
	uint16 result;

	//int c;
	//int timeout = 0;

	if (v3CommBuf.txhead==v3CommBuf.txtail) return;  // no data to send;
   
   // Read up to _max_packet_size characters from local buffer
	while (len < _max_packet_size) 
   {
      data[len++] = v3CommBuf.tx[v3CommBuf.txtail++];
      //v3CommBuf.txtail &= V3BUFMASK;  // not needed for 256 size buffer with U8 index
      if (v3CommBuf.txhead==v3CommBuf.txtail) break;
	}

	if (len > 0) {
		// Stack may return "out-of-memory" error if the local buffer is full -> in that case, just keep trying until the command succeeds
		do {
#if VECTTOR_V3
			result = gecko_cmd_gatt_server_send_characteristic_notification(_conn_handle, gattdb_gatt_spp_data, len, data)->result;
#else
			result = gecko_cmd_gatt_server_send_characteristic_notification(_conn_handle, gattdb_notifyChar, len, data)->result;
#endif
			_sCounters.num_writes++;
		} while(result == bg_err_out_of_memory);

		if (result != 0) {
			printLog("Unexpected error: %x, %s:%d\r\n", result, __func__, __LINE__);
		} else {
			_sCounters.num_pack_sent++;
			_sCounters.num_bytes_sent += len;
		}
	}
}

extern void I2C_Sensors(void);
/**
 * @brief  SPP server mode main loop
 */
void spp_server_main(void)
{
int i;
//char logout[80];
U8 sectic = TIC_TIMER_PERSEC;

   //SLEEP_SleepBlockBegin(sleepEM2); // Disable sleeping

  // Create one-shot soft timer to delay for I/O expander powerup
//  gecko_cmd_hardware_set_soft_timer(TIC_TIMER_CONST/TIC_TIMER_PERSEC, OS_TIMER_HANDLE, true);

  //RBG OTA tesing
  // Create soft timer to Handle init I/O and runtime I/O
  gecko_cmd_hardware_set_soft_timer(TIC_TIMER_CONST/TIC_TIMER_PERSEC, OS_TIMER_HANDLE, false);

  while (1) {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;

   //bpt_main();   // For Bio-Sensor estimation -
#if HR_SPO2_HRV
    data_report_execute();
#endif
#if 0
    if(calibration_success)
    {
    	calibration_success = false;
    	DSInterface_BuildCommand_itself();
    }
#endif
    if(_main_state == STATE_SPP_MODE)
    {
    	/* If SPP data mode is active, use non-blocking gecko_peek_event() */
    	evt = gecko_peek_event();

    	if(evt == NULL)
    	{
    		/* No stack events to be handled -> send data from local TX buffer */
    		//send_spp_data();
#if VECTTOR_V3
		#if USE_OF_QUEUE
			BLE_TransferMesSamplesFromQueue();
        #else
			send_spp_msg();                             // send V3 Message, if any from circular buffer
		#endif
    	    recv_spp_msg();                             // receive V3 Message, if any from circular buffer
#else
    		BLE_TransferMesSamplesFromQueue();
#endif
    		continue;  		                            // Jump directly to next iteration i.e. call gecko_peek_event() again
    	}
    }
    else
    {
//      /* if there are no events pending then the next call to gecko_wait_event() may cause
//      * device go to deep sleep. Make sure that debug prints are flushed before going to sleep */
//      if (!gecko_event_pending()){flushLog();}

    	/* SPP data mode not active -> check for stack events using the blocking API */
    	evt = gecko_wait_event();
    }

    /* Handle events */
   switch (BGLIB_MSG_ID(evt->header)) 
   {

      /* This boot event is generated when the system boots up after reset.
       * Here the system is set to start advertising immediately after boot procedure. */
      case gecko_evt_system_boot_id:

         reset_variables();
         gecko_cmd_gatt_set_max_mtu(247);
         gecko_cmd_le_gap_start_advertising(HANDLE_V3, le_gap_general_discoverable, le_gap_undirected_connectable);
        
#if 0  //include this code for Eddystone

        // eddystone additions
        //gecko_cmd_system_set_tx_power(0);

        /* Set 8 dBm Transmit Power */
         gecko_cmd_le_gap_set_advertise_tx_power(HANDLE_EDDYSTONE, 75);

        /* Set advertising parameters. 1000ms advertisement interval.
         * The first parameter is advertising set handle
         * The next two parameters are minimum and maximum advertising interval, both in
         * units of (milliseconds * 1.6).
         * The last two parameters are duration and maxevents left as default. */
        gecko_cmd_le_gap_set_advertise_timing(HANDLE_EDDYSTONE, 1600, 1600, 0, 0);
        gecko_cmd_le_gap_bt5_set_adv_data(HANDLE_EDDYSTONE, 0, EDDYSTONE_DATA_LEN, eddystone_data);
        /* Start general advertising. */
        gecko_cmd_le_gap_start_advertising(HANDLE_EDDYSTONE, le_gap_user_data, le_gap_non_connectable);

        /* Start non-connectable advertising ibeacons */
        bcnSetupAdvBeaconing();
#endif
         //iBeacon support 
         bcnSetupAdvBeaconing();
      break;


    	/* Connection opened event */
      case gecko_evt_le_connection_opened_id:

         _conn_handle = evt->data.evt_le_connection_opened.connection;
         printBleDebug("Connected\r\n");
         _main_state = STATE_CONNECTED;
         v3status.spp = STATE_CONNECTED;

         /* Request connection parameter update.
          * conn.interval min 20ms, max 40ms, slave latency 4 intervals,
          * supervision timeout 2 seconds
          * (These should be compliant with Apple Bluetooth Accessory Design Guidelines, both R7 and R8) */
         gecko_cmd_le_connection_set_timing_parameters(_conn_handle, 24, 40, 0, 200, 0, 0xFFFF);
    	break;

      case gecko_evt_le_connection_parameters_id:
         printBleDebug("Conn.parameters: interval %u units, txsize %u\r\n", evt->data.evt_le_connection_parameters.interval, evt->data.evt_le_connection_parameters.txsize);
    	break;

      case gecko_evt_gatt_mtu_exchanged_id:
         /* Calculate maximum data per one notification / write-without-response, this depends on the MTU.
          * up to ATT_MTU-3 bytes can be sent at once  */
         _max_packet_size = evt->data.evt_gatt_mtu_exchanged.mtu - 3;
         _min_packet_size = _max_packet_size; /* Try to send maximum length packets whenever possible */
         printBleDebug("MTU exchanged: %d\r\n", evt->data.evt_gatt_mtu_exchanged.mtu);
    	break;

      case gecko_evt_le_connection_closed_id:
         printLog("DISCONNECTED!\r\n");

#if VECTTOR_V3
         /* Show statistics (RX/TX counters) after disconnect: */
         printStats(&_sCounters);

         reset_variables();
#else
         queue_reset(&BLEQUEUE);
#endif
        /* Check if need to boot to OTA DFU mode */
        if (boot_to_dfu) {
          /* Enter to OTA DFU mode */
          gecko_cmd_system_reset(2);
        } else {
          /* Restart advertising after client has disconnected */
          gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
          //SLEEP_SleepBlockEnd(sleepEM2); // Enable sleeping
        }
#if HR_SPO2_HRV
        NVIC_SystemReset();
#endif
         /* Restart advertising */
         //gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_undirected_connectable);
    	break;

      /* Check if the user-type OTA Control Characteristic was written.
       * If ota_control was written, boot the device into Device Firmware Upgrade (DFU) mode. */
      case gecko_evt_gatt_server_user_write_request_id:

        if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
          /* Set flag to enter to OTA mode */
          boot_to_dfu = 1;
          /* Send response to Write Request */
          //gecko_cmd_gatt_server_send_user_write_response(
          //  evt->data.evt_gatt_server_user_write_request.connection,
          //  gattdb_ota_control,
          //  bg_err_success);

          /* Close connection to enter to DFU OTA mode */
            gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
          //printBleDebug("gattdb_ota_control: 0x%x, line:%d\r\n", evt->data.evt_gatt_server_user_write_request.value.data[0], __LINE__);
        }
        else
        {
        	printBleDebug("unknown: 0x%x, line:%d\r\n", evt->data.evt_gatt_server_user_write_request.value.data[0], __LINE__);
        }
        /* Send response to Write Request */
        gecko_cmd_gatt_server_send_user_write_response(evt->data.evt_gatt_server_user_write_request.connection,
        		                                       evt->data.evt_gatt_server_user_write_request.characteristic,
													   bg_err_success);
        break;

      case gecko_evt_gatt_server_characteristic_status_id:
      {
         struct gecko_msg_gatt_server_characteristic_status_evt_t *pStatus;
         pStatus = &(evt->data.evt_gatt_server_characteristic_status);

#if VECTTOR_V3
         if (pStatus->characteristic == gattdb_gatt_spp_data)
#else
         if (pStatus->characteristic == gattdb_notifyChar)
#endif
         {
            if (pStatus->status_flags == gatt_server_client_config) {
               // Characteristic client configuration (CCC) for spp_data has been changed
               if (pStatus->client_config_flags == gatt_notification) {
                  _main_state = STATE_SPP_MODE;
                  v3status.spp = STATE_SPP_MODE;
                  //gecko_cmd_hardware_set_soft_timer(32768, TIC_TIMER_HANDLE, 0);
                  //SLEEP_SleepBlockBegin(sleepEM2); // Disable sleeping
                  //initBoard();
                  printBleDebug("SPP Mode ON\r\n");
               } else {
                  printBleDebug("SPP Mode OFF\r\n");
                  _main_state = STATE_CONNECTED;
                  v3status.spp = STATE_CONNECTED;
                  //SLEEP_SleepBlockEnd(sleepEM2); // Enable sleeping
               }
    		}
    	}
    }
    break;

   case gecko_evt_gatt_server_attribute_value_id:
   {
       if(evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_configRWChar)
       {
    	   int len;

    	   len = evt->data.evt_gatt_server_attribute_value.value.len;
    	   printBleDebug("gattdb_configRWChar: 0x%x, line:%d\r\n", evt->data.evt_gatt_server_attribute_value.value.data[0], __LINE__);

    	   printBleDebug("BLE_I[Line:%d]: data RECV: len=%d, data-->", __LINE__, len);
    	   for(int x=0; x < len; x++)
    	   {
    		 //if ((BLE_DS_INTERFACE != NULL) && (params->data[x] != 0))
    		   if (evt->data.evt_gatt_server_attribute_value.value.data[x] != 0)
    		   {
    			   DSInterface_BuildCommand((char)evt->data.evt_gatt_server_attribute_value.value.data[x]);
    		   }
    		   //if(params->data[x] == 0)
    		   //    printf("00");
    		   //else if(params->data[x] < 16)
    		   //    printf("0%x", params->data[x]);
    		   //else
    		   //    printf("%x", params->data[x]);
    	   }
    	   //printf("\n\r");                                                          // Commented by Jason
    	   //for(x=0; x < len; x++)                                                     // Added by Jason
    	   //{
    	   //  printf("%c", evt->data.evt_gatt_server_attribute_value.value.data[x]); // Added by Jason
    	   //}                                                                          // Added by Jason
    	   //printf("\n\r");                                                            // Added by Jason
       }
       else if(evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_gatt_spp_data)
       {
    	   for(i=0; i<evt->data.evt_gatt_server_attribute_value.value.len; i++)
    	   {
    		   v3CommBuf.rx[v3CommBuf.rxhead++]  = evt->data.evt_gatt_server_attribute_value.value.data[i];
    		   if(v3CommBuf.rxhead == v3CommBuf.rxtail)
    		   {
    			   // make error condition for receive buffer overflow
    			   break;
    		   }
    	   }
    	   _sCounters.num_pack_received++;
    	   _sCounters.num_bytes_received += evt->data.evt_gatt_server_attribute_value.value.len;
    	   printBleDebug("gattdb_gatt_spp_data: 0x%x, line:%d\r\n", evt->data.evt_gatt_server_attribute_value.value.data[0], __LINE__);
       }
       else
       {
    	   printBleDebug("Unknown_config: 0x%x, line:%d\r\n", evt->data.evt_gatt_server_attribute_value.value.data[0], __LINE__);
       }
   }
   break;

     /* Software Timer event */
   case gecko_evt_hardware_soft_timer_id:

      switch (evt->data.evt_hardware_soft_timer.handle) 
      {
         //case TIC_TIMER_HANDLE:  // currently once per second
            //v3_state(); // sequence main V3 state machine
         //break;

         case OS_TIMER_HANDLE:
        
         // Keep v3_state call before LED and feedback calls to make the response more immediate
         sectic++;
         
         I2C_Sensors();  // determine if biosensor present and manage power and control
                         // also handles biosensor sensor reset and initialization

         if (sectic >= TIC_TIMER_PERSEC)
         {
            v3_state(); // sequence main V3 state machine once per second
            sectic = 0;
         }

         ledseq();  // step the LED player
         fbseq(); // Step the feedback player (Haptic and buzzer)
         
       
         #if 0  //New method Make conditional on sensor present
         //Jason // For Bio-Sensor estimation - 
         if((v3status.spp == STATE_CONNECTED)||(v3status.spp == STATE_SPP_MODE))  bpt_main();
         else  bpt_main_reset();
         #endif
         
         if ((v3status.conn & v3PINSBIO)==0) bpt_main();  // call if biosensor is sensor present

         if (v3sleep.sleepsec) 
         {
        	v3_state(); // sequence main V3 state machine
        	gecko_cmd_hardware_set_soft_timer(0, OS_TIMER_HANDLE, false);  // turn off timer
            gecko_cmd_hardware_set_soft_timer((v3sleep.sleepsec*TIC_TIMER_CONST), OS_TIMER_HANDLE, false);  // set new sleep timer
            v3sleep.sleepsec = 0;  // clear flag 
            sectic = TIC_TIMER_PERSEC;   // Force state machine to be called next entry
         }

         break;

         default:
         break;
      }
   break;

   //break;

   default:
   break;
   }
  }
}

void bcnSetupAdvBeaconing(void)
{

  /* This function sets up a custom advertisement package according to iBeacon specifications.
   * The advertisement package is 30 bytes long. See the iBeacon specification for further details.
   */

  static struct
  {
    uint8_t flagsLen; /* Length of the Flags field. */
    uint8_t flagsType; /* Type of the Flags field. */
    uint8_t flags; /* Flags field. */
    uint8_t mandataLen; /* Length of the Manufacturer Data field. */
    uint8_t mandataType; /* Type of the Manufacturer Data field. */
    uint8_t compId[2]; /* Company ID field. */
    uint8_t beacType[2]; /* Beacon Type field. */
    uint8_t uuid[16]; /* 128-bit Universally Unique Identifier (UUID). The UUID is an identifier for the company using the beacon*/
    uint8_t majNum[2]; /* Beacon major number. Used to group related beacons. */
    uint8_t minNum[2]; /* Beacon minor number. Used to specify individual beacons within a group.*/
    uint8_t txPower; /* The Beacon's measured RSSI at 1 meter distance in dBm. See the iBeacon specification for measurement guidelines. */
  } bcnBeaconAdvData = {
  /* Flag bits - See Bluetooth 4.0 Core Specification , Volume 3, Appendix C, 18.1 for more details on flags. */
  2, /* length  */
  0x01, /* type */
  0x04 | 0x02, /* Flags: LE General Discoverable Mode, BR/EDR is disabled. */

  /* Manufacturer specific data */
  26, /* length of field*/
  0xFF, /* type of field */

  /* The first two data octets shall contain a company identifier code from
   * the Assigned Numbers - Company Identifiers document */
  /* 0x004C = Apple */
  {UINT16_TO_BYTES(0x004C)},

  /* Beacon type */
  /* 0x0215 is iBeacon */
  {UINT16_TO_BYTE1(0x0215), UINT16_TO_BYTE0(0x0215)},

  /* 128 bit / 16 byte UUID */
  {0xE2, 0xC5, 0x6D, 0xB5, 0xDF, 0xFB, 0x48, 0xD2, 0xB0, 0x60, 0xD0, 0xF5, 0xA7, 0x10, 0x96, 0xE0},

  /* Beacon major number */
  /* Set to 34987 and converted to correct format */
  {UINT16_TO_BYTE1(34987), UINT16_TO_BYTE0(34987)},

  /* Beacon minor number */
  /* Set as 1025 and converted to correct format */
  {UINT16_TO_BYTE1(1025), UINT16_TO_BYTE0(1025)},

  /* The Beacon's measured RSSI at 1 meter distance in dBm */
  /* 0xC3 is -61dBm */
  0xC3};

  //
  uint8_t len = sizeof(bcnBeaconAdvData);
  uint8_t *pData = (uint8_t*)(&bcnBeaconAdvData);

  /* Set custom advertising data */

  gecko_cmd_le_gap_bt5_set_adv_data(HANDLE_IBEACON, 0, len, pData);

  /* Set 8 dBm Transmit Power */
  gecko_cmd_le_gap_set_advertise_tx_power(HANDLE_IBEACON, 75);

  /* Set advertising parameters. 200ms (320/1.6) advertisement interval. All channels used.
   * The first two parameters are minimum and maximum advertising interval, both in
   * units of (milliseconds * 1.6).  */
  gecko_cmd_le_gap_set_advertise_timing(HANDLE_IBEACON, 1600, 1600, 0, 0);

  /* Start advertising in user mode */
  gecko_cmd_le_gap_start_advertising(HANDLE_IBEACON, le_gap_user_data, le_gap_non_connectable);

}
