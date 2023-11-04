/**
  ******************************************************************************
  * @file    sx127x.h
  * @author  Maciej Janc
  * @version V1.0
  * @date    03-October-2023
  * @brief   sx127x library header.
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SX127X_H
#define SX127X_H

#include "stm32f1xx.h"
#include <stdbool.h>

#ifdef __cplusplus
	extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum
{
	SX127X_STATUS_OK = 0,
	SX127X_STATUS_GENERAL_ERROR,
	SX127X_STATUS_HAL_SPI_ERROR,
	SX127X_STATUS_BUSY_TRANSMITTING,
	SX127X_STATUS_WRONG_INPUT,
	SX127X_STATUS_NOT_IMPLEMENTED,
	SX127X_STATUS_CRC_ERROR,
} sx127x_result_t;

typedef enum
{
	SX127X_LNA_GAIN = 1,
	SX127X_LNA_GAIN_06DB,
	SX127X_LNA_GAIN_12DB,
	SX127X_LNA_GAIN_24DB,
	SX127X_LNA_GAIN_36DB,
	SX127X_LNA_GAIN_48DB,
} sx127x_lna_boost_gain_t;

typedef enum
{
	SX127X_OP_MODE_SLEEP = 0,
	SX127X_OP_MODE_STANDBY,
	SX127X_OP_MODE_FS_TX,
	SX127X_OP_MODE_TX,
	SX127X_OP_MODE_FS_RX,
	SX127X_OP_MODE_RX,
	SX127X_OP_MODE_RX_SINGLE,
	SX127X_OP_MODE_CAD,
} sx127x_op_mode_t;

typedef enum
{
	SX127X_OP_MODE_RANGE_FSK_OOK = 0,
	SX127X_OP_MODE_RANGE_LORA,
} sx127x_op_mode_range_t;

typedef enum
{
	SX127X_PA_SELECT_RFO_PIN = 0,
	SX127X_PA_SELECT_PA_BOOST_PIN,
} sx127x_pa_select_t;

typedef enum
{
	SX127X_OP_MODE_EXPLICIT_HEADER = 0,
	SX127X_OP_MODE_IMPLICIT_HEADER,
} sx127x_header_mode_t;

typedef enum
{
	SX127X_LOW_FREQUENCY_MODE = 0,
	SX127X_HIGH_FREQUENCY_MODE,
} sx127x_frequency_mode_t;

typedef enum
{
	SX127X_SPREADING_FACTOR_6 = 6,
	SX127X_SPREADING_FACTOR_7,
	SX127X_SPREADING_FACTOR_8,
	SX127X_SPREADING_FACTOR_9,
	SX127X_SPREADING_FACTOR_10,
	SX127X_SPREADING_FACTOR_11,
	SX127X_SPREADING_FACTOR_12,
} sx127x_spreading_factor_t;

typedef enum
{
	SX127X_BANDWIDTH_KHZ_7_8 = 0,
	SX127X_BANDWIDTH_KHZ_10_4,
	SX127X_BANDWIDTH_KHZ_15_6,
	SX127X_BANDWIDTH_KHZ_20_8,
	SX127X_BANDWIDTH_KHZ_31_25,
	SX127X_BANDWIDTH_KHZ_41_7,
	SX127X_BANDWIDTH_KHZ_62_5,
	SX127X_BANDWIDTH_KHZ_125,
	SX127X_BANDWIDTH_KHZ_250,
	SX127X_BANDWIDTH_KHZ_500,
} sx127x_bandwidth_khz_t;

typedef enum
{
	SX127X_CODING_RATE_4_5 = 1,
	SX127X_CODING_RATE_4_6,
	SX127X_CODING_RATE_4_7,
	SX127X_CODING_RATE_4_8,
} sx127x_coding_rate_t;

typedef struct
{
	void* spi_handler;
	uint32_t spi_timeout_ms;
} sx127x_spi_hal_t;

typedef struct
{
	sx127x_spi_hal_t* spi_hal;
	sx127x_result_t (*spi_read_register_function)(sx127x_spi_hal_t*, const uint8_t, int8_t*);
	sx127x_result_t (*spi_write_register_function)(sx127x_spi_hal_t*, const uint8_t, uint8_t);
	void (*delay_function)(uint32_t);
	void (*reset_function)(void);
} sx127x_spi_configuration_t;

typedef struct
{
	sx127x_spi_configuration_t* spi_conf;
	sx127x_op_mode_range_t op_mode_range;
	sx127x_lna_boost_gain_t lna_boost_gain;
	sx127x_spreading_factor_t spreading_factor;
	sx127x_bandwidth_khz_t bandwidth_khz;
	sx127x_frequency_mode_t frequency_mode;
	sx127x_coding_rate_t coding_rate;
	bool is_rx_payload_crc_on;
	uint32_t frequency_hz;
	bool is_auto_agc_on;
	uint8_t tx_power_level_dbm;
	sx127x_pa_select_t pa_output_pin;
} sx127x_configuration_t;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

#define SX127X_REG_FIFO							0x00
#define SX127X_REG_OP_MODE						0x01
#define SX127X_REG_VERSION						0x42
#define SX127X_REG_FRF_MSB              		0x06
#define SX127X_REG_FRF_MID              		0x07
#define SX127X_REG_FRF_LSB              		0x08
#define SX127X_REG_LNA                  		0x0C
#define SX127X_REG_MODEM_CONFIG_3       		0x26
#define SX127X_REG_PA_CONFIG					0x09
#define SX127X_REG_PACKET_RSSI_VALUE            0x1A
#define SX127X_REG_PACKET_SNR_VALUE            	0x19
#define SX127X_REG_IRQ_FLAGS            		0x12
#define SX127X_REG_FIFO_ADDR_PTR        		0x0D
#define SX127X_REG_FIFO_TX_BASE_ADDR    		0x0E
#define SX127X_REG_FIFO_RX_BASE_ADDR    		0x0F
#define SX127X_REG_FIFO_RX_CURRENT_ADDRESS		0x10
#define SX127X_REG_PAYLOAD_LENGTH    			0x22
#define SX127X_REG_RX_BYTES_NUMBER 				0x13
#define SX127X_REG_OCP							0x0B
#define SX127X_REG_MODEM_CONFIG2 				0x1E
#define	SX127X_REG_MODEM_CONFIG1				0x1D
#define SX127X_REG_PA_DAC						0x4D

#define SX127X_PA_BOOST                		 	0x80
#define SX127X_PA_MAX_DAC_VALUE                	0x07
#define SX127X_PA_DEFAULT_DAC_VALUE             0x04

#define SX127X_IRQ_TX_DONE_MASK           		0x08
#define SX127X_IRQ_RX_DONE_MASK					0x40
#define SX127X_IRQ_CRC_ERROR_MASK 				0x20

#define SX127X_FXOSC 							32000000
/* Exported functions ------------------------------------------------------- */

sx127x_result_t sx127x_lora_init(sx127x_configuration_t* sx127x_conf);
sx127x_result_t sx127x_lora_receive_data(sx127x_spi_configuration_t* spi_conf, uint8_t* data, uint8_t data_size, uint8_t timeout_ms);
sx127x_result_t sx127x_lora_send_data(sx127x_spi_configuration_t* spi_conf, uint8_t* data, uint8_t data_size, uint8_t timeout_ms);
sx127x_result_t sx127x_lora_get_rssi(sx127x_configuration_t* sx127x_conf, uint8_t* rssi);
sx127x_result_t sx127x_lora_get_snr(sx127x_configuration_t* sx127x_conf, uint8_t* snr);

#ifdef __cplusplus
}
#endif

#endif /* SX127X_H */

 /**** M. Janc ******/
 /****END OF FILE****/
