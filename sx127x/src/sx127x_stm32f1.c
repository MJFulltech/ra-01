/**
  ******************************************************************************
  * @file    sx127x_stm32f1.c
  * @author  Maciej Janc
  * @version V1.0
  * @date    03-October-2023
  * @brief   sx127x mapper for stm32f1 library.
  ******************************************************************************
*/

/*
 * Here please implement your platform dependent file
 * SPI read and write functions
 *
 */

#include "sx127x_stm32f1.h"

void config_spi(SPI_HandleTypeDef* spi)
{
	 __HAL_RCC_GPIOA_CLK_ENABLE();
	 __HAL_RCC_GPIOB_CLK_ENABLE();
	 __HAL_RCC_SPI1_CLK_ENABLE();

	 GPIO_InitTypeDef gpio_no_pullup;
	 gpio_no_pullup.Mode = GPIO_MODE_AF_PP;
	 gpio_no_pullup.Pin = GPIO_PIN_5 | GPIO_PIN_7;			// SCK, MOSI
	 gpio_no_pullup.Pull = GPIO_NOPULL;
	 gpio_no_pullup.Speed = GPIO_SPEED_FREQ_HIGH;
	 HAL_GPIO_Init(GPIOA, &gpio_no_pullup);

	 GPIO_InitTypeDef gpio_in;
	 gpio_in.Mode = GPIO_MODE_AF_INPUT;
	 gpio_in.Pin = GPIO_PIN_6;								// MISO
	 gpio_in.Speed = GPIO_SPEED_FREQ_HIGH;
	 HAL_GPIO_Init(GPIOA, &gpio_in);

	 GPIO_InitTypeDef gpio_out;
	 gpio_out.Mode = GPIO_MODE_OUTPUT_PP;
	 gpio_out.Pin = GPIO_PIN_4;								// CS (NSS)
	 HAL_GPIO_Init(GPIOA, &gpio_out);
	 gpio_out.Pin = GPIO_PIN_0;								// RESET
	 HAL_GPIO_Init(GPIOB, &gpio_out);

	 spi->Instance = SPI1;
	 spi->Init.Mode = SPI_MODE_MASTER;
	 spi->Init.FirstBit = SPI_FIRSTBIT_MSB;
	 spi->Init.NSS = SPI_NSS_SOFT;
	 spi->Init.CLKPolarity = SPI_POLARITY_LOW;
	 spi->Init.CLKPhase = SPI_PHASE_1EDGE;
	 spi->Init.NSS = SPI_NSS_SOFT;
	 spi->Init.DataSize = SPI_DATASIZE_8BIT;
	 spi->Init.Direction = SPI_DIRECTION_2LINES;
	 spi->Init.TIMode = SPI_TIMODE_DISABLE;
	 spi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	 spi->Init.CRCPolynomial = 10;
	 spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;	// 1MHz

	 HAL_SPI_Init(spi);

	 __HAL_SPI_ENABLE(spi);
}

void config_uart(UART_HandleTypeDef* uart)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();

	GPIO_InitTypeDef gpio;
	gpio.Mode = GPIO_MODE_AF_PP;
	gpio.Pin = GPIO_PIN_2;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &gpio);

	gpio.Mode = GPIO_MODE_AF_INPUT;
	gpio.Pin = GPIO_PIN_3;
	HAL_GPIO_Init(GPIOA, &gpio);

	uart->Instance = USART2;
	uart->Init.BaudRate = 115200;
	uart->Init.WordLength = UART_WORDLENGTH_8B;
	uart->Init.Parity = UART_PARITY_NONE;
	uart->Init.StopBits = UART_STOPBITS_1;
	uart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uart->Init.OverSampling = UART_OVERSAMPLING_16;
	uart->Init.Mode = UART_MODE_TX_RX;
	HAL_UART_Init(uart);
}

void config_irq(UART_HandleTypeDef* uart)
{
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef gpio;
	gpio.Mode = GPIO_MODE_IT_RISING;
	gpio.Pull = GPIO_PULLUP;
	gpio.Pin = GPIO_PIN_1;
	HAL_GPIO_Init(GPIOB, &gpio);

	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

sx127x_result_t sx127x_read(SPI_HandleTypeDef* spi, const uint8_t address, int8_t* read_data, uint8_t read_size, uint32_t timeout)
{
	sx127x_result_t result = SX127X_STATUS_OK;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); 	// CS (NSS)
	result = (sx127x_result_t)HAL_SPI_Transmit(spi, &address, 1, timeout);
	if (result != SX127X_STATUS_OK)
	{
		return SX127X_STATUS_HAL_SPI_ERROR;
	}
	while (HAL_SPI_GetState(spi) != HAL_SPI_STATE_READY);
	result = (sx127x_result_t)HAL_SPI_Receive(spi, read_data, read_size, timeout);
	if (result != SX127X_STATUS_OK)
	{
		return SX127X_STATUS_HAL_SPI_ERROR;
	}
	while (HAL_SPI_GetState(spi) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	return result;
}

sx127x_result_t sx127x_write(SPI_HandleTypeDef* spi, const uint8_t address, uint8_t* write_data, uint8_t write_size, uint32_t timeout)
{
	sx127x_result_t result = SX127X_STATUS_OK;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); 	// CS (NSS)
	result = (sx127x_result_t)HAL_SPI_Transmit(spi, &address, 1, timeout);
	if (result != SX127X_STATUS_OK)
	{
		return SX127X_STATUS_HAL_SPI_ERROR;
	}
	while (HAL_SPI_GetState(spi) != HAL_SPI_STATE_READY);
	result = (sx127x_result_t)HAL_SPI_Transmit(spi, write_data, write_size, timeout);
	if (result != SX127X_STATUS_OK)
	{
		return SX127X_STATUS_HAL_SPI_ERROR;
	}
	while (HAL_SPI_GetState(spi) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	return result;
}

sx127x_result_t sx127x_read_registers(SPI_HandleTypeDef* spi, uint8_t address, int8_t* read_data, uint8_t read_size)
{
	return sx127x_read(spi, address & 0x7f, read_data, read_size, 2000);
}

sx127x_result_t sx127x_write_registers(SPI_HandleTypeDef* spi, const uint8_t address, uint8_t* write_data, uint8_t write_size)
{
	return sx127x_write(spi, address | 0x80, write_data, write_size, 2000);
}

sx127x_result_t sx127x_read_register(sx127x_spi_hal_t* spi, const uint8_t address, uint8_t* result)
{
	return sx127x_read(spi->spi_handler, address & 0x7f, result, 1, spi->spi_timeout_ms);
}

sx127x_result_t sx127x_write_register(sx127x_spi_hal_t* spi, const uint8_t address, uint8_t value)
{
	return sx127x_write(spi->spi_handler, address | 0x80, &value, 1, spi->spi_timeout_ms);
}

void sx127x_reset(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); 	// CS (NSS)
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_Delay(500);
}
