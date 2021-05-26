/***************************************************************************//**
 * @file
 * @brief hal-config-board.h
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#ifndef HAL_CONFIG_BOARD_H
#define HAL_CONFIG_BOARD_H

#include "em_device.h"
#include "hal-config-types.h"

/***************************************************************************//**
 * @file
 * @brief hal-config-standalone-default.h
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

// $[CMU]
#define BSP_CLK_LFXO_PRESENT                          (0)
#define BSP_CLK_HFXO_PRESENT                          (1)
#define BSP_CLK_LFXO_INIT                              CMU_LFXOINIT_DEFAULT
#define BSP_CLK_LFXO_CTUNE                            (79U)
#define BSP_CLK_LFXO_FREQ                             (32768U)
#define BSP_CLK_HFXO_FREQ                             (38400000UL)
#define BSP_CLK_HFXO_CTUNE                            (129)
#define BSP_CLK_HFXO_INIT                              CMU_HFXOINIT_DEFAULT
#define BSP_CLKOUT0_PIN                 			(4U)
#define BSP_CLKOUT0_PORT                			(gpioPortD)
// [CMU]$

#warning "Following pin mappings need to be set for your custom board when using printf..>"

#if 0
// $[SERIAL]
#define BSP_SERIAL_APP_PORT                           (HAL_SERIAL_PORT_USART0)
#define BSP_SERIAL_APP_CTS_PIN                        (4U)
#define BSP_SERIAL_APP_CTS_PORT                       (gpioPortA)

#define BSP_SERIAL_APP_RX_PIN                         (6U)
#define BSP_SERIAL_APP_RX_PORT                        (gpioPortA)

#define BSP_SERIAL_APP_TX_PIN                         (5U)
#define BSP_SERIAL_APP_TX_PORT                        (gpioPortA)

#define BSP_SERIAL_APP_RTS_PIN                        (1U)
#define BSP_SERIAL_APP_RTS_PORT                       (gpioPortC)

// [SERIAL]$

// $[UARTNCP]
#define BSP_UARTNCP_USART_PORT                        (HAL_SERIAL_PORT_USART0)
#define BSP_UARTNCP_CTS_PIN                           (4U)
#define BSP_UARTNCP_CTS_PORT                          (gpioPortA)

#define BSP_UARTNCP_RX_PIN                            (6U)
#define BSP_UARTNCP_RX_PORT                           (gpioPortA)

#define BSP_UARTNCP_TX_PIN                            (5U)
#define BSP_UARTNCP_TX_PORT                           (gpioPortA)

#define BSP_UARTNCP_RTS_PIN                           (1U)
#define BSP_UARTNCP_RTS_PORT                          (gpioPortC)

// [UARTNCP]$
#endif

// $[USART0]
#define I2S0_BLK_PIN                           (0U)
#define I2S0_BLK_PORT                          (gpioPortD)

#define I2S0_LRCLK_PIN                         (1U)
#define I2S0_LRCLK_PORT                        (gpioPortD)

#define I2S0_MISO_PIN                          (2U)
#define I2S0_MISO_PORT                         (gpioPortD)

#define I2S0_MOSI_PIN                          (3U)
#define I2S0_MOSI_PORT                         (gpioPortD)

// $[USART2]
#define I2S1_BLK_PIN                           (0U)
#define I2S1_BLK_PORT                          (gpioPortC)

#define I2S1_LRCLK_PIN                         (1U)
#define I2S1_LRCLK_PORT                        (gpioPortC)

#define I2S1_MISO_PIN                          (2U)
#define I2S1_MISO_PORT                         (gpioPortC)

#define I2S1_MOSI_PIN                          (3U)
#define I2S1_MOSI_PORT                         (gpioPortC)


#if 0
#define PORTIO_USART0_CTS_PIN                         (4U)
#define PORTIO_USART0_CTS_PORT                        (gpioPortA)

#define PORTIO_USART0_RTS_PIN                         (1U)
#define PORTIO_USART0_RTS_PORT                        (gpioPortC)

#define PORTIO_USART0_RX_PIN                          (6U)
#define PORTIO_USART0_RX_PORT                         (gpioPortA)

#define PORTIO_USART0_TX_PIN                          (5U)
#define PORTIO_USART0_TX_PORT                         (gpioPortA)

#define BSP_USART0_CTS_PIN                            (4U)
#define BSP_USART0_CTS_PORT                           (gpioPortA)

#define BSP_USART0_RX_PIN                             (6U)
#define BSP_USART0_RX_PORT                            (gpioPortA)

#define BSP_USART0_TX_PIN                             (5U)
#define BSP_USART0_TX_PORT                            (gpioPortA)

#define BSP_USART0_RTS_PIN                            (1U)
#define BSP_USART0_RTS_PORT                           (gpioPortC)
#endif
// [USART0]$

#define BSP_TRACE_SWO_PORT								(gpioPortA)
#define BSP_TRACE_SWO_PIN								(3U)

#define BSP_I2C0_SDA_PORT								(gpioPortA)
#define BSP_I2C0_SDA_PIN								(6U)

#define BSP_I2C0_SCL_PORT								(gpioPortA)
#define BSP_I2C0_SCL_PIN								(5U)

#endif /* HAL_CONFIG_BOARD_H */
