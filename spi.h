 /*
 * SPI.h
 *
 *  Created on: 6/27/2020
 *      Author: rgraczyk
 */
#ifndef SPI_H_
#define SPI_H_


#define SPIDRV_MASTER_USART1                                          \
  {                                                                    \
    USART1,                     /* USART port                       */ \
    gpioPortB,                  /* USART Tx port location number    */ \
    gpioPortB,                  /* USART Rx port location number    */ \
    gpioPortA,                  /* USART Clk port location number   */ \
    gpioPortA,                  /* USART Cs port location number    */ \
    1,                          /* USART Tx pin location number    */ \
    0,                          /* USART Rx pin location number    */ \
    0,                          /* USART Clk pin location number    */ \
    4,                          /* USART Cs pin location number     */ \
    16000000,                   /* Bitrate                          */ \
    8,                          /* Frame length                     */ \
    0,                          /* Dummy tx value for rx only funcs */ \
    spidrvMaster,               /* SPI mode                         */ \
    spidrvBitOrderMsbFirst,     /* Bit order on bus                 */ \
    spidrvClockMode0,           /* SPI clock/phase mode             */ \
    spidrvCsControlAuto,        /* CS controlled by the driver      */ \
    spidrvSlaveStartImmediate   /* Slave start transfers immediately*/ \
  }

extern bool spi_erase_active;
  
extern void initSPI(void); 
extern void spi_cmd_ce(void);
extern bool spi_cmd_busy(void);
extern void spi_cmd_test(void);
extern void spi_cmd_read(U16 address,  union v3_message_UNION * pv3msgU);
extern void spi_cmd_write(U16 * address, union v3_message_UNION * pv3msgU);
extern U16 spi_get_free(void);

#endif /* SPI_H_*/
