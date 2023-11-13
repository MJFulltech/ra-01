/**
  ******************************************************************************
  * @file    sx127x.c
  * @author  Maciej Janc
  * @version V1.0
  * @date    03-October-2023
  * @brief   sx127x library.
  ******************************************************************************
*/

#include "sx127x.h"

sx127x_result_t sx127x_lora_set_ocp(sx127x_spi_configuration_t* spi_conf, uint8_t current_ma)
{
	sx127x_result_t result = SX127X_STATUS_OK;
	uint8_t ocp_trim;
	uint8_t reg_ocp;

	if (current_ma <= 120)
	{
		ocp_trim = (current_ma - 45) / 5;
	}
	else if (current_ma <= 240)
	{
		ocp_trim = (current_ma) / 10;
	}
	else
	{
		return SX127X_STATUS_WRONG_INPUT;
	}

	result = (*spi_conf->spi_read_register_function)(spi_conf->spi_hal, SX127X_REG_OCP, &reg_ocp);
	if (result != SX127X_STATUS_OK) return result;

	reg_ocp &= 0xC0;
	// SET OCP ON
	reg_ocp |= ((uint8_t)1 << 5);
	reg_ocp &= ocp_trim;

	result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_OCP, reg_ocp);
	return result;
}

sx127x_result_t sx127x_set_op_mode_range(sx127x_spi_configuration_t* spi_conf, sx127x_op_mode_range_t op_mode_range)
{
	uint8_t reg_op_mode;
	sx127x_result_t result = SX127X_STATUS_OK;
	result = (*spi_conf->spi_read_register_function)(spi_conf->spi_hal, SX127X_REG_OP_MODE, &reg_op_mode);
	if (result != SX127X_STATUS_OK) return result;

	reg_op_mode &= 0x7F;
	reg_op_mode |= ((uint8_t)op_mode_range << 7);
	result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_OP_MODE, reg_op_mode);
	return result;
}

sx127x_result_t sx127x_lora_set_lna_boost_gain(sx127x_spi_configuration_t* spi_conf, sx127x_lna_boost_gain_t lna_boost_gain)
{
	uint8_t reg_lna;
	sx127x_result_t result = SX127X_STATUS_OK;
	result = (*spi_conf->spi_read_register_function)(spi_conf->spi_hal, SX127X_REG_LNA, &reg_lna);
	if (result != SX127X_STATUS_OK) return result;

	reg_lna &= 0x1F;
	reg_lna |= ((uint8_t)lna_boost_gain << 5);
	result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_LNA, reg_lna);
	return result;
}

sx127x_result_t sx127x_lora_zero_tx_rx_registers(sx127x_spi_configuration_t* spi_conf)
{
	sx127x_result_t result = SX127X_STATUS_OK;
	result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_FIFO_TX_BASE_ADDR, 0);
	if (result != SX127X_STATUS_OK) return result;

	result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_FIFO_RX_BASE_ADDR, 0);
	return result;
}

sx127x_result_t sx127x_lora_set_rx_payload_crc(sx127x_spi_configuration_t* spi_conf, bool is_rx_payload_crc_on)
{
	sx127x_result_t result = SX127X_STATUS_OK;
	uint8_t reg_config2;

	result = (*spi_conf->spi_read_register_function)(spi_conf->spi_hal, SX127X_REG_MODEM_CONFIG2, &reg_config2);
	if (result != SX127X_STATUS_OK) return result;

	reg_config2 &= 0xFB;
	reg_config2 |= ((uint8_t)is_rx_payload_crc_on << 2);
	result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_MODEM_CONFIG2, reg_config2);
	return result;
}

sx127x_result_t sx127x_lora_set_frequency(sx127x_spi_configuration_t* spi_conf, uint32_t frequency_hz)
{
	sx127x_result_t result = SX127X_STATUS_OK;
	// f_step = f_xosc / (pow(2,19))
	// f_rf = f_step / frequency_hz
	uint64_t carrier_frequency = ((uint64_t)frequency_hz << 19) / SX127X_FXOSC;
	uint8_t f_rf[3] = {(uint8_t)(carrier_frequency >> 16),
						(uint8_t)(carrier_frequency >> 8),
						(uint8_t)(carrier_frequency)};
	result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_FRF_MSB, f_rf[0]);
	if (result != SX127X_STATUS_OK) return result;

	result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_FRF_MID, f_rf[1]);
	if (result != SX127X_STATUS_OK) return result;

	result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_FRF_LSB, f_rf[2]);
	return result;
}

// TODO: remove magic numbers and fix flag setting
sx127x_result_t sx127x_lora_set_tx_power(sx127x_spi_configuration_t* spi_conf, uint8_t tx_output_power_dbm, sx127x_pa_select_t pa_output_pin)
{
	sx127x_result_t result = SX127X_STATUS_OK;
	if (pa_output_pin == 0)
	{
		if (tx_output_power_dbm < 0 || tx_output_power_dbm > 14)
		{
			return SX127X_STATUS_WRONG_INPUT;
		}
		// Select max output power and pin output = 0
		tx_output_power_dbm |= 0x70;
		result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_PA_CONFIG, tx_output_power_dbm);
		if (result != SX127X_STATUS_OK) return result;
	}
	else
	{
		if (tx_output_power_dbm > 20)
		{
			return SX127X_STATUS_WRONG_INPUT;
		}
		if (tx_output_power_dbm > 17)
		{
			// Ease mapping value
			tx_output_power_dbm -= 3;

			uint8_t reg_pa_dac;
			result = (*spi_conf->spi_read_register_function)(spi_conf->spi_hal, SX127X_REG_PA_DAC, &reg_pa_dac);
			if (result != SX127X_STATUS_OK) return result;
			reg_pa_dac &= 0xF8;
			reg_pa_dac |= SX127X_PA_MAX_DAC_VALUE;
			result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_PA_DAC, reg_pa_dac);
			if (result != SX127X_STATUS_OK) return result;

			result = sx127x_lora_set_ocp(spi_conf, 140);
			if (result != SX127X_STATUS_OK) return result;
		}
		else
		{
			// Ease substracting value
			if (tx_output_power_dbm < 2)
			{
				tx_output_power_dbm = 2;
			}
			uint8_t reg_pa_dac;
			result = (*spi_conf->spi_read_register_function)(spi_conf->spi_hal, SX127X_REG_PA_DAC, &reg_pa_dac);
			if (result != SX127X_STATUS_OK) return result;

			reg_pa_dac &= 0xF8;
			reg_pa_dac |= SX127X_PA_DEFAULT_DAC_VALUE;
			result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_PA_DAC, reg_pa_dac);
			if (result != SX127X_STATUS_OK) return result;

			result = sx127x_lora_set_ocp(spi_conf, 100);
			if (result != SX127X_STATUS_OK) return result;
		}
		result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_PA_CONFIG, SX127X_PA_BOOST | (tx_output_power_dbm - 2));
	}
	return result;
}

sx127x_result_t sx127x_set_op_mode(sx127x_spi_configuration_t* spi_conf, sx127x_op_mode_t op_mode)
{
	sx127x_result_t result = SX127X_STATUS_OK;
	uint8_t reg_op_mode;
	result = (*spi_conf->spi_read_register_function)(spi_conf->spi_hal, SX127X_REG_OP_MODE, &reg_op_mode);
	if (result != SX127X_STATUS_OK) return result;

	reg_op_mode &= 0xF8;
	reg_op_mode |= op_mode;
	result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_OP_MODE, reg_op_mode);

	return result;
}

sx127x_result_t sx127x_lora_get_is_rx_done(sx127x_spi_configuration_t* spi_conf, bool* result_is_rx_done)
{
	sx127x_result_t result = SX127X_STATUS_OK;
	uint8_t reg_irq_flags;
	result = (*spi_conf->spi_read_register_function)(spi_conf->spi_hal, SX127X_REG_IRQ_FLAGS, &reg_irq_flags);
	*result_is_rx_done = (bool)(reg_irq_flags & SX127X_IRQ_RX_DONE_MASK);
	return result;
}

sx127x_result_t sx127x_lora_set_auto_agc(sx127x_spi_configuration_t* spi_conf, bool is_auto_agc_on)
{
	sx127x_result_t result = SX127X_STATUS_OK;
	uint8_t reg_config3;
	result = (*spi_conf->spi_read_register_function)(spi_conf->spi_hal, SX127X_REG_MODEM_CONFIG_3, &reg_config3);
	if (result != SX127X_STATUS_OK) return result;

	reg_config3 &= 0xFB;
	reg_config3 |= ((uint8_t)is_auto_agc_on << 2);
	result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_MODEM_CONFIG_3, reg_config3);
	return result;
}

sx127x_result_t sx127x_lora_set_header_mode(sx127x_spi_configuration_t* spi_conf, sx127x_header_mode_t header_mode)
{
	sx127x_result_t result = SX127X_STATUS_OK;
	uint8_t reg_config1;
	result = (*spi_conf->spi_read_register_function)(spi_conf->spi_hal, SX127X_REG_MODEM_CONFIG1, &reg_config1);
	if (result != SX127X_STATUS_OK) return result;

	reg_config1 &= 0xFE;
	reg_config1 |= ((uint8_t)header_mode);
	result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_MODEM_CONFIG1, reg_config1);
	return result;
}

sx127x_result_t sx127x_get_is_in_mode(sx127x_spi_configuration_t* spi_conf, sx127x_op_mode_t op_mode, bool* return_is_in_mode)
{
	sx127x_result_t result = SX127X_STATUS_OK;
	uint8_t reg_op_mode;
	result = (*spi_conf->spi_read_register_function)(spi_conf->spi_hal, SX127X_REG_OP_MODE, &reg_op_mode);
	if (result != SX127X_STATUS_OK) return result;

	*return_is_in_mode = (bool)((reg_op_mode & op_mode) == op_mode);
	return result;
}

sx127x_result_t sx127x_lora_clear_irq(sx127x_spi_configuration_t* spi_conf, uint8_t flag_mask)
{
	return (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_IRQ_FLAGS, flag_mask);
}

sx127x_result_t sx127x_lora_get_is_transmitting(sx127x_spi_configuration_t* spi_conf, bool* result_is_transmitting)
{
	sx127x_result_t result = SX127X_STATUS_OK;
	uint8_t reg_op_mode;
	uint8_t reg_irq_flags;
	bool is_in_tx_mode;

	result = sx127x_get_is_in_mode(spi_conf, SX127X_OP_MODE_TX, &is_in_tx_mode);
	if (result != SX127X_STATUS_OK) return result;

	if (is_in_tx_mode)
	{
		*result_is_transmitting = true;
	}

	result = (*spi_conf->spi_read_register_function)(spi_conf->spi_hal, SX127X_REG_IRQ_FLAGS, &reg_irq_flags);
	if (result != SX127X_STATUS_OK) return result;

	if (reg_irq_flags & SX127X_IRQ_TX_DONE_MASK)
	{
		result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_IRQ_FLAGS, SX127X_IRQ_TX_DONE_MASK);
	}

	return result;
}

sx127x_result_t sx127x_lora_send_data(sx127x_spi_configuration_t* spi_conf, uint8_t* data, uint8_t data_size, uint8_t timeout_ms)
{
	sx127x_result_t result = SX127X_STATUS_OK;
	bool is_transmitting = false;

	sx127x_lora_get_is_transmitting(spi_conf, &is_transmitting);

	if (is_transmitting)
	{
		return result;
	}

	result = sx127x_set_op_mode(spi_conf, SX127X_OP_MODE_STANDBY);
	if (result != SX127X_STATUS_OK) return result;

	result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_FIFO_ADDR_PTR, 0);
	if (result != SX127X_STATUS_OK) return result;
	result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_PAYLOAD_LENGTH, 0);
	if (result != SX127X_STATUS_OK) return result;

	if (data_size > 255)
	{
		return SX127X_STATUS_WRONG_INPUT;
	}

	for (size_t i = 0; i < data_size; i++)
	{
		result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_FIFO, data[i]);
		if (result != SX127X_STATUS_OK) return result;
	}

	result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_PAYLOAD_LENGTH, data_size);
	if (result != SX127X_STATUS_OK) return result;

	result = sx127x_set_op_mode(spi_conf, SX127X_OP_MODE_TX);
	if (result != SX127X_STATUS_OK) return result;

	uint8_t reg_irq_flags = 0x00;

	while ((reg_irq_flags & SX127X_IRQ_TX_DONE_MASK) == 0)
	{
		(*spi_conf->delay_function)(10);
		result = (*spi_conf->spi_read_register_function)(spi_conf->spi_hal, SX127X_REG_IRQ_FLAGS, &reg_irq_flags);
		if (result != SX127X_STATUS_OK) return result;

	}
	result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_IRQ_FLAGS, SX127X_IRQ_TX_DONE_MASK);
	return result;
}

sx127x_result_t sx127x_lora_get_is_crc_error(sx127x_spi_configuration_t* spi_conf, bool* result_is_crc_error)
{
	sx127x_result_t result = SX127X_STATUS_OK;
	uint8_t reg_irq_flags;
	result = (*spi_conf->spi_read_register_function)(spi_conf->spi_hal, SX127X_REG_IRQ_FLAGS, &reg_irq_flags);
	*result_is_crc_error = (bool)(reg_irq_flags & SX127X_IRQ_CRC_ERROR_MASK);
	return result;
}

sx127x_result_t sx127x_lora_get_rx_payload_crc(sx127x_spi_configuration_t* spi_conf, bool* is_rx_payload_crc_on)
{
	sx127x_result_t result = SX127X_STATUS_OK;
	uint8_t reg_config2;
	result = (*spi_conf->spi_read_register_function)(spi_conf->spi_hal, SX127X_REG_MODEM_CONFIG2, &reg_config2);
	*is_rx_payload_crc_on = (bool)(reg_config2 & 0x40);
	return result;
}

sx127x_result_t sx127x_lora_receive_data(sx127x_spi_configuration_t* spi_conf, uint8_t* data, uint8_t data_size, uint8_t timeout_ms)
{
	sx127x_result_t result = SX127X_STATUS_OK;
	uint8_t bytes_to_read;
	uint8_t fifo_rx_address;
	uint8_t min_bytes_to_read = 0;
	bool is_rx_done;
	bool is_crc_in_payload;
	bool is_crc_error;

	result = sx127x_set_op_mode(spi_conf, SX127X_OP_MODE_STANDBY);
	if (result != SX127X_STATUS_OK) return result;

	result = sx127x_lora_get_is_rx_done(spi_conf, &is_rx_done);
	if (result != SX127X_STATUS_OK) return result;

	if (is_rx_done)
	{
		result = sx127x_lora_get_rx_payload_crc(spi_conf, &is_crc_in_payload);
		if (result != SX127X_STATUS_OK) return result;

		if (is_crc_in_payload)
		{
			result = sx127x_lora_get_is_crc_error(spi_conf, &is_crc_error);
			if (result != SX127X_STATUS_OK) return result;
			if (is_crc_error)
			{
				return SX127X_STATUS_CRC_ERROR;
			}
		}
		result = sx127x_lora_clear_irq(spi_conf, 0xFF);
		if (result != SX127X_STATUS_OK) return result;

		result = (*spi_conf->spi_read_register_function)(spi_conf->spi_hal, SX127X_REG_RX_BYTES_NUMBER, &bytes_to_read);
		if (result != SX127X_STATUS_OK) return result;

		result = (*spi_conf->spi_read_register_function)(spi_conf->spi_hal, SX127X_REG_FIFO_RX_CURRENT_ADDRESS, &fifo_rx_address);
		if (result != SX127X_STATUS_OK) return result;

		result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_FIFO_ADDR_PTR, fifo_rx_address);
		if (result != SX127X_STATUS_OK) return result;

		min_bytes_to_read = data_size <= bytes_to_read ? data_size : bytes_to_read;

		for (int i = 0; i < min_bytes_to_read; i++)
		{
			result = (*spi_conf->spi_read_register_function)(spi_conf->spi_hal, SX127X_REG_FIFO, &data[i]);
			if (result != SX127X_STATUS_OK) return result;
		}
	}
	result = sx127x_set_op_mode(spi_conf, SX127X_OP_MODE_RX);
	return result;
}

sx127x_result_t sx127x_lora_read_data(sx127x_spi_configuration_t* spi_conf, uint8_t* data, uint8_t data_size, uint8_t timeout_ms)
{
	sx127x_result_t result = SX127X_STATUS_OK;
	uint8_t bytes_to_read;
	uint8_t fifo_rx_address;
	uint8_t min_bytes_to_read = 0;
	bool is_rx_done;
	bool is_crc_in_payload;
	bool is_crc_error;


	result = sx127x_lora_get_rx_payload_crc(spi_conf, &is_crc_in_payload);
	if (result != SX127X_STATUS_OK) return result;

	if (is_crc_in_payload)
	{
		result = sx127x_lora_get_is_crc_error(spi_conf, &is_crc_error);
		if (result != SX127X_STATUS_OK) return result;
		if (is_crc_error)
		{
			return SX127X_STATUS_CRC_ERROR;
		}
	}

	result = sx127x_lora_clear_irq(spi_conf, 0xFF);
	if (result != SX127X_STATUS_OK) return result;

	result = (*spi_conf->spi_read_register_function)(spi_conf->spi_hal, SX127X_REG_RX_BYTES_NUMBER, &bytes_to_read);
	if (result != SX127X_STATUS_OK) return result;

	result = (*spi_conf->spi_read_register_function)(spi_conf->spi_hal, SX127X_REG_FIFO_RX_CURRENT_ADDRESS, &fifo_rx_address);
	if (result != SX127X_STATUS_OK) return result;

	result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_FIFO_ADDR_PTR, fifo_rx_address);
	if (result != SX127X_STATUS_OK) return result;

	min_bytes_to_read = data_size <= bytes_to_read ? data_size : bytes_to_read;

	for (int i = 0; i < min_bytes_to_read; i++)
	{
		result = (*spi_conf->spi_read_register_function)(spi_conf->spi_hal, SX127X_REG_FIFO, &data[i]);
		if (result != SX127X_STATUS_OK) return result;
	}
	return result;
}

sx127x_result_t sx127x_lora_set_spreading_factor(sx127x_spi_configuration_t* spi_conf, sx127x_spreading_factor_t spreading_factor)
{
	sx127x_result_t result = SX127X_STATUS_OK;
	uint8_t reg_config2;

	result = (*spi_conf->spi_read_register_function)(spi_conf->spi_hal, SX127X_REG_MODEM_CONFIG2, &reg_config2);
	if (result != SX127X_STATUS_OK) return result;

	reg_config2 &= 0x0F;
	reg_config2 |= ((uint8_t)spreading_factor << 4);
	result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_MODEM_CONFIG2, reg_config2);
	return result;
}

sx127x_result_t sx127x_lora_set_bandwidth(sx127x_spi_configuration_t* spi_conf, sx127x_bandwidth_khz_t bandwidth_khz)
{
	sx127x_result_t result = SX127X_STATUS_OK;
	uint8_t reg_config1;

	result = (*spi_conf->spi_read_register_function)(spi_conf->spi_hal, SX127X_REG_MODEM_CONFIG1, &reg_config1);
	if (result != SX127X_STATUS_OK) return result;

	reg_config1 &= 0x0F;
	reg_config1 |= ((uint8_t)bandwidth_khz << 4);
	result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_MODEM_CONFIG1, reg_config1);
	return result;
}

sx127x_result_t sx127x_lora_set_frequency_mode(sx127x_spi_configuration_t* spi_conf, sx127x_frequency_mode_t frequency_mode)
{
	sx127x_result_t result = SX127X_STATUS_OK;
	uint8_t reg_op_mode;

	result = (*spi_conf->spi_read_register_function)(spi_conf->spi_hal, SX127X_REG_OP_MODE, &reg_op_mode);
	if (result != SX127X_STATUS_OK) return result;

	reg_op_mode &= 0xF7;
	reg_op_mode |= ((uint8_t)frequency_mode << 3);
	result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_OP_MODE, reg_op_mode);
	return result;
}

sx127x_result_t sx127x_lora_set_coding_rate(sx127x_spi_configuration_t* spi_conf, sx127x_coding_rate_t coding_rate)
{
	sx127x_result_t result = SX127X_STATUS_OK;
	uint8_t reg_config1;

	result = (*spi_conf->spi_read_register_function)(spi_conf->spi_hal, SX127X_REG_MODEM_CONFIG1, &reg_config1);
	if (result != SX127X_STATUS_OK) return result;

	reg_config1 &= 0xF1;
	reg_config1 |= ((uint8_t)coding_rate << 1);
	result = (*spi_conf->spi_write_register_function)(spi_conf->spi_hal, SX127X_REG_MODEM_CONFIG1, reg_config1);
	return result;
}

sx127x_result_t sx127x_lora_get_packet_rssi(sx127x_configuration_t* sx127x_conf, uint8_t* rssi)
{
	sx127x_result_t result = SX127X_STATUS_OK;
	uint8_t reg_rssi;
	result = (*sx127x_conf->spi_conf->spi_read_register_function)(sx127x_conf->spi_conf->spi_hal, SX127X_REG_PACKET_RSSI_VALUE, &reg_rssi);
	if (sx127x_conf->frequency_mode == SX127X_HIGH_FREQUENCY_MODE)
	{
		*rssi = 157 - reg_rssi * 1.0667;
	}
	else
	{
		*rssi = 164 - reg_rssi * 1.0667;
	}
	return result;
}

sx127x_result_t sx127x_lora_get_packet_snr(sx127x_configuration_t* sx127x_conf, uint8_t* snr)
{
	sx127x_result_t result = SX127X_STATUS_OK;
	uint8_t reg_snr;
	result = (*sx127x_conf->spi_conf->spi_read_register_function)(sx127x_conf->spi_conf->spi_hal, SX127X_REG_PACKET_SNR_VALUE, &reg_snr);
	*snr = reg_snr / 4;
	return result;
}

sx127x_result_t sx127x_lora_init(sx127x_configuration_t* sx127x_conf)
{
	sx127x_result_t result = SX127X_STATUS_OK;

	do
	{
		result = sx127x_set_op_mode(sx127x_conf->spi_conf, SX127X_OP_MODE_SLEEP);
		if (result != SX127X_STATUS_OK) break;

		result = sx127x_set_op_mode_range(sx127x_conf->spi_conf, sx127x_conf->op_mode_range);
		if (result != SX127X_STATUS_OK) break;

		result = sx127x_lora_set_header_mode(sx127x_conf->spi_conf, sx127x_conf->header_mode);
		if (result != SX127X_STATUS_OK) return result;

		result = sx127x_lora_set_frequency(sx127x_conf->spi_conf, sx127x_conf->frequency_hz);
		if (result != SX127X_STATUS_OK) break;

		result = sx127x_lora_zero_tx_rx_registers(sx127x_conf->spi_conf);
		if (result != SX127X_STATUS_OK) break;

		result = sx127x_lora_set_lna_boost_gain(sx127x_conf->spi_conf, sx127x_conf->lna_boost_gain);
		if (result != SX127X_STATUS_OK) break;

		result = sx127x_lora_set_auto_agc(sx127x_conf->spi_conf, sx127x_conf->is_auto_agc_on);
		if (result != SX127X_STATUS_OK) break;

		result = sx127x_lora_set_tx_power(sx127x_conf->spi_conf, sx127x_conf->tx_power_level_dbm, sx127x_conf->pa_output_pin);
		if (result != SX127X_STATUS_OK) break;

		result = sx127x_lora_set_spreading_factor(sx127x_conf->spi_conf, sx127x_conf->spreading_factor);
		if (result != SX127X_STATUS_OK) break;

		result = sx127x_lora_set_rx_payload_crc(sx127x_conf->spi_conf, sx127x_conf->is_rx_payload_crc_on);
		if (result != SX127X_STATUS_OK) break;

		result = sx127x_lora_set_bandwidth(sx127x_conf->spi_conf, sx127x_conf->bandwidth_khz);
		if (result != SX127X_STATUS_OK) break;

		result = sx127x_lora_set_coding_rate(sx127x_conf->spi_conf, sx127x_conf->coding_rate);
		if (result != SX127X_STATUS_OK) break;

		result = sx127x_lora_set_frequency_mode(sx127x_conf->spi_conf, sx127x_conf->frequency_mode);
		if (result != SX127X_STATUS_OK) break;

		result = sx127x_set_op_mode(sx127x_conf->spi_conf, SX127X_OP_MODE_STANDBY);
		if (result != SX127X_STATUS_OK) break;
	} while(0);

	return result;
}
