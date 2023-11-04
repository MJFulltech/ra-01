/**
  ******************************************************************************
  * @file    main.c
  * @author  Maciej Janc
  * @version V1.0
  * @date    03-October-2023
  * @brief   Default main function.
  ******************************************************************************
*/

#include "stm32f1xx.h"
#include <sx127x.h>
#include <sx127x_stm32f1.h>

#define MAX_DATA_SIZE 100

SPI_HandleTypeDef spi1;
UART_HandleTypeDef uart;

void uart_send_string(char* str)
{
	HAL_UART_Transmit(&uart, (uint8_t*)str, strlen(str), 1000);
}

void uart_send_char_array(uint8_t* char_ptr, uint8_t size)
{
	HAL_UART_Transmit(&uart, (uint8_t*)char_ptr, size, 1000);
}

void uart_send_number(uint8_t uint8_number)
{
	uint8_t val[3];
	if (uint8_number < 10)
	{
		val[0] = (uint8_t)uint8_number + '0';
		HAL_UART_Transmit(&uart, (uint8_t*)&val[0], 1, 1000);
	}
	else if (uint8_number < 100)
	{
		val[0] = (uint8_t)(((uint8_t)uint8_number / (uint8_t)10) + '0');
		val[1] = (uint8_t)(((uint8_t)uint8_number % (uint8_t)10) + '0');
		HAL_UART_Transmit(&uart, (uint8_t*)&val[0], 2, 1000);
	}
	else
	{
		val[0] = (uint8_t)(((uint8_t)uint8_number / (uint8_t)100) + '0');
		val[1] = (uint8_t)((((uint8_t)uint8_number / (uint8_t) 10) % (uint8_t)10) + '0');
		val[2] = (uint8_t)(((uint8_t)uint8_number % (uint8_t)10) + '0');
		HAL_UART_Transmit(&uart, (uint8_t*)&val[0], 3, 1000);
	}
}

int main(void)
{
	sx127x_result_t result = SX127X_STATUS_OK;
	SystemCoreClock = 8000000;
	HAL_Init();
	HAL_Delay(100);
	config_spi(&spi1);
	HAL_Delay(100);
	config_uart(&uart);
	HAL_Delay(100);

	sx127x_spi_hal_t spi_hal;
	spi_hal.spi_handler = &spi1;
	spi_hal.spi_timeout_ms = 2000;

	sx127x_spi_configuration_t sx127x_spi_conf;
	sx127x_spi_conf.spi_read_register_function = &sx127x_read_register;
	sx127x_spi_conf.spi_write_register_function = &sx127x_write_register;
	sx127x_spi_conf.delay_function = &HAL_Delay;
	sx127x_spi_conf.reset_function = &sx127x_reset;
	sx127x_spi_conf.spi_hal = &spi_hal;

	sx127x_configuration_t sx127x_conf;
	sx127x_conf.spi_conf = &sx127x_spi_conf;
	sx127x_conf.op_mode_range = SX127X_OP_MODE_RANGE_LORA;
	sx127x_conf.lna_boost_gain = SX127X_LNA_GAIN_12DB;
	sx127x_conf.is_rx_payload_crc_on = true;
	sx127x_conf.bandwidth_khz = SX127X_BANDWIDTH_KHZ_125;
	sx127x_conf.spreading_factor = SX127X_SPREADING_FACTOR_8;
	sx127x_conf.coding_rate = SX127X_CODING_RATE_4_6;
	sx127x_conf.frequency_hz = 434000000;
	sx127x_conf.frequency_mode = SX127X_LOW_FREQUENCY_MODE;
	sx127x_conf.is_auto_agc_on = true;
	sx127x_conf.tx_power_level_dbm = 13;
	sx127x_conf.pa_output_pin = SX127X_PA_SELECT_PA_BOOST_PIN;

	uart_send_string("Start Ra-01 receiver\r\n");

	(sx127x_conf.spi_conf->reset_function)();
	HAL_Delay(1000);

	uint8_t version;
	result = (sx127x_conf.spi_conf->spi_read_register_function)(sx127x_conf.spi_conf->spi_hal, SX127X_REG_VERSION, &version);
	if (result != SX127X_STATUS_OK)
	{
		uart_send_string("Error reading SX127x version, exiting program ...");
		return -1;
	}
	else if (version != 0x12)
	{
		uart_send_string("Error reading SX127x version - not equals 0x12, exiting program ...\r\n");
		NVIC_SystemReset();
	}

	result = sx127x_lora_init(&sx127x_conf);
	if (result != SX127X_STATUS_OK)
	{
		uart_send_string("Error initializing SX127x, exiting program ...\r\n");
		return -1;
	}

	HAL_Delay(1000);

	uint8_t data[MAX_DATA_SIZE];
	memset(data, 0, MAX_DATA_SIZE);

	uint8_t rssi;
	uint8_t snr;

	while(1)
	{
		HAL_Delay(1000);
		result = sx127x_lora_receive_data(sx127x_conf.spi_conf, &data[0], (uint8_t)MAX_DATA_SIZE, 2000);
		if (result != SX127X_STATUS_OK)
		{
			uart_send_string("Error receiving data\r\n");
		}
		result = sx127x_lora_get_rssi(&sx127x_conf, &rssi);
		if (result != SX127X_STATUS_OK)
		{
			uart_send_string("Error getting RSSI\r\n");
		}
		result = sx127x_lora_get_snr(&sx127x_conf, &snr);
		if (result != SX127X_STATUS_OK)
		{
			uart_send_string("Error getting SNR\r\n");
		}
		else
		{
			uart_send_string("\r\n/******************************/\r\n");
			uart_send_string("Received data:\r\n");
			uart_send_char_array(&data[0], MAX_DATA_SIZE);
			uart_send_string("\r\n Last received packet RSSI [dBm]: ");
			uart_send_number(rssi);
			uart_send_string("\r\nLast received packet SNR [dB]: ");
			uart_send_number(snr);
			uart_send_string("\r\n/******************************/\r\n");
		}
	}
}
