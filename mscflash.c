/******************************************************************************
 * @file
 * @brief V3 PROCESSOR FLASH routines
 * @author Ron Graczyk
 * @version 
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2020 Artaflex
 *******************************************************************************
 *  05/1/2020
 *
 ******************************************************************************/
 
#include <stdint.h>
#include "global.h"
#include "v3.h"
#include "stdbool.h"
#include "mscflash.h"
#include "em_se.h"

//#include "em_msc.h"
#include <string.h>
 
#define USER_DATA USERDATA_BASE  // flash area for user storage

uint32_t *puserdata  = (uint32_t*) USER_DATA;


const struct statedata flashrom = MSC_INIT_DEFAULT;

struct statedata progdata;
 
void flash_init(bool wipe)
{
struct statedata *pflashdata;
//MSC_ExecConfig_TypeDef execConfig = MSC_EXECCONFIG_DEFAULT;

//   MSC_ExecConfigSet(&execConfig);
//   MSC_Init();

   pflashdata = (struct statedata *) puserdata;
	if ((pflashdata->key != MSC_KEY) || wipe )  // init user data page if first time or wipe set
	{
		SE_eraseUserData();
      //MSC_ErasePage (puserdata);
		//MSC_WriteWord (puserdata, &flashrom, (uint32_t) sizeof(flashrom));
      SE_writeUserData(0,&flashrom, (uint32_t) sizeof(flashrom));
	}
	
	memcpy(&progdata, puserdata, sizeof (flashrom));   // copy flash to ram
	
}

void flash_write(void)
{
	SE_eraseUserData();
	SE_writeUserData(0, &progdata, (uint32_t) sizeof(progdata));	
}


