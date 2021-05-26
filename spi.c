 /*
 * spi.c
 *
 *  Created on: 6/29/2020
 *      Author: rgraczyk
 *
 * using a MX25R6435F part via SPI bus, fixed 128 byte segments for each message
 *
 */
#include "global.h"
#include "stdbool.h"
#include "string.h"
#include "spidrv.h"
#include "v3.h"
#include "spi.h"
#include "MX25_CMD.h"
#include "MAX14676E.h"

static SPIDRV_HandleData_t handleData;
static SPIDRV_Handle_t handle = &handleData;

bool spi_erase_active = false;

bool spi_cmd_res(void);
void spi_cmd_rdsr(uint8_t *StatusReg);
void spi_cmd_wren(void);

void initSPI(void)
{
  SPIDRV_Init_t initData = SPIDRV_MASTER_USART1;
   
  // Initialize an SPI driver instance.
  SPIDRV_Init(handle, &initData);   
  spi_erase_active = false;
   
}


void TransferComplete(SPIDRV_Handle_t handle,
                      Ecode_t transferStatus,
                      int itemsTransferred)
{
  if (transferStatus == ECODE_EMDRV_SPIDRV_OK) {
   // Success !
  }
}


bool spi_cmd_res(void) 
{
U8 txBuffer[5] = { FLASH_CMD_RES, 0, 0, 0,0};
U8 rxbuffer[5];
   
   SPIDRV_MTransferB(handle, txBuffer, rxbuffer, 5);
   
   if(rxbuffer[4] == FLASH_VAL_RES) return FLASH_OK;
      else return FLASH_ERR;

}


void spi_cmd_rdsr(uint8_t *StatusReg)
{
U8 txBuffer[3] = { FLASH_CMD_RDSR, 0,0};
U8 rxbuffer[3];
   
   SPIDRV_MTransferB(handle, txBuffer, rxbuffer, 2);   
   *StatusReg = rxbuffer[1];
   
}


void spi_cmd_wren(void)
{
U8 txBuffer[1] = {FLASH_CMD_WREN};

   SPIDRV_MTransmitB(handle, txBuffer,1);   
   
}



bool spi_cmd_busy(void)
{
U8 status;  
   
   spi_cmd_rdsr(&status);
   if (status & FLASH_WIP_MASK )  return true;
   return false;
   
}

void spi_cmd_read(U16 address,  union v3_message_UNION * pv3msgU)
{
U8 txBuffer[V3_FLASHBUF_SIZ];
U8 rxbuffer[V3_FLASHBUF_SIZ]; 
U8 len;

   txBuffer[0] = FLASH_CMD_READ ;
   txBuffer[1] = (U8) (address>>9);
   txBuffer[2] = (U8) ((address & 0x1FE)>>1);
   txBuffer[3] = (U8) ((address & 0x01)<<7);
   
 
   SPIDRV_MTransferB(handle, txBuffer, rxbuffer, V3_FLASH_CMD_SIZ+V3_LEN_POS+1);  // get message size
   len = rxbuffer[V3_FLASH_CMD_SIZ+V3_LEN_POS];
   
   if (len<V3_HDR_SIZE || len>V3_MAX_SIZE) len = V3_HDR_SIZE;  // error or reading unwritten flash
                                                // change this to an error condition later
   
   SPIDRV_MTransferB(handle, txBuffer, rxbuffer, len+V3_FLASH_CMD_SIZ); // get full message
   memcpy(pv3msgU->v3_buf8, rxbuffer+V3_FLASH_CMD_SIZ, len);

}


void spi_cmd_write(U16 * address, union v3_message_UNION * pv3msgU)
{
U8 txBuffer[V3_FLASHBUF_SIZ];
//U8 rxbuffer[V3_FLASHBUF_SIZ];
U8 len;

   
   if (spi_erase_active) return;
   if (*address == V3_NO_HANDLE) return;  // do not store until init message is received
                                         // also will return if memory is full

   txBuffer[0] = FLASH_CMD_PP;
   txBuffer[1] = (U8) (*address>>9);
   txBuffer[2] = (U8) ((*address & 0x1FE)>>1);
   txBuffer[3] = (U8) ((*address & 0x01)<<7);
   
   len = pv3msgU->v3msg.len;
   if (len<V3_HDR_SIZE || len>V3_MAX_SIZE) len = V3_HDR_SIZE;  // error 
                                            // change this to an error condition later   
   memcpy(txBuffer+V3_FLASH_CMD_SIZ,pv3msgU->v3_buf8, len);
   
   len+=V3_FLASH_CMD_SIZ;
   
   while(spi_cmd_busy()){}; 
   spi_cmd_wren();
   SPIDRV_MTransmitB(handle, txBuffer,len); // write flash
   while(spi_cmd_busy()){};
   (*address)++;  // increment Handle
   //v3info.reports = v3Handle;
   v3info.reports = *address;
}

//flash chip erase command
void spi_cmd_ce(void)
{
//U8 status;
U8 txBuffer[1] = {FLASH_CMD_CE};

   while(spi_cmd_busy()){};
   spi_cmd_wren();
   SPIDRV_MTransmitB(handle, txBuffer,1);  // CHIP ERASE
   //while(spi_cmd_busy()){};

   // make flag to signal erase in progress unil erase is complete , check busy in v3 state
   spi_erase_active = true;
}


U8 spi_cmd_first(U16 address)
{
U8 txBuffer[V3_FLASH_CMD_SIZ+1];
U8 rxbuffer[V3_FLASH_CMD_SIZ+1]; 
//U8 len;

   txBuffer[0] = FLASH_CMD_READ ;
   txBuffer[1] = (U8) (address>>9);
   txBuffer[2] = (U8) ((address & 0x1FE)>>1);
   txBuffer[3] = (U8) ((address & 0x01)<<7);
   
 
   SPIDRV_MTransferB(handle, txBuffer, rxbuffer, V3_FLASH_CMD_SIZ+1);  // get message size
   return (rxbuffer[V3_FLASH_CMD_SIZ]);  // first byte of message
}


// get pointer to next free flash location.
// optimize with sort routine later
U16 spi_get_free(void)
{
U16 address;

   for (address = 0;address < V3_FLASH_END;address++) if (spi_cmd_first(address)==0xFF) break;

   return (address);

}


