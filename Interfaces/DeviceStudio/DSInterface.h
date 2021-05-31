/*
 * DSInterface.h
 *
 *  Created on: May 28, 2021
 *      Author: jason Chen Local
 */

#ifndef DEVICESTUDIO_DSINTERFACE_H_
#define DEVICESTUDIO_DSINTERFACE_H_

#define COMM_SUCCESS        0
#define COMM_GENERAL_ERROR  -1
#define COMM_INVALID_PARAM  -254
#define COMM_NOT_RECOGNIZED -255

#define FLASH_ERR_GENERAL   -1
#define FLASH_ERR_CHECKSUM  -2
#define FLASH_ERR_AUTH      -3

#define DS_MAX_NUM_SENSORCOMMS	8

#define DS_BINARY_PACKET_START_BYTE	0xAA


void DSInterface_BuildCommand(char ch);







#endif /* DEVICESTUDIO_DSINTERFACE_H_ */
