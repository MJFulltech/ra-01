/**
  ******************************************************************************
  * @file    sx127x_stm32f1.h
  * @author  Maciej Janc
  * @version V1.0
  * @date    03-October-2023
  * @brief   sx127x mapper for stm32f1 library.
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SX127X_STM32F1_H
#define SX127X_STM32F1_H

#include "stm32f1xx.h"
#include "sx127x.h"
#include <stdbool.h>

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void config_spi(SPI_HandleTypeDef* spi);
void config_uart(UART_HandleTypeDef* uart);
sx127x_result_t sx127x_read_register(sx127x_spi_hal_t* spi_conf, uint8_t address, uint8_t* result);
sx127x_result_t sx127x_write_register(sx127x_spi_hal_t* spi_conf, uint8_t address, uint8_t value);
void sx127x_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* SX127X_STM32F1_H */

/**** M. Janc ******/
/****END OF FILE****/
