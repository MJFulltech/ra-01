# Ra-01 example code and SX1278 library

## Contents

The repository contains example code for the Ra-01 module, named "STM32F1_ra01_<sample name>". These are projects created in STM32 Workbench with its own STM32F103C8T6 board definition (STM32F103C8T6_board.xml).

The sx127x folder contains a library with definitions of functions, types and registers to support the SX1278 module located on Ra-01. This folder can be built separately or linked.
The sx127x_stm32f1.c file contains the implementation of platform-specific SPI, UART, IRQ and SX1278 reset functions.

I also added the HAL library that I used.

## Work principle

The finished project is a simple example in which you configure the module options via the sx127x_configuration_t structure. Then example (depending on the project) works as a receiver or transmitter. Please note that some of the available configuration options also require the configuration of specific registers (https://cdn-shop.adafruit.com/product-files/3179/sx1276_77_78_79.pdf).

## Wiring

| STM32F103C8T6 | Ra-01         |
| ------------- |:-------------:|
| B0            | RST           |
| A7            | MOSI          |
| A6            | MISO          |
| A5            | SCK           |
| A4            | NSS           |

| STM32F103C8T6 | Serial adapter|
| ------------- |:-------------:|
| A3            | TX            |
| A2            | RX            |

## Establishing communication

### Frequency

Registers from 0x06 to 0x08 are used to set carrier frequency. Carrier frequency in LoRa term is central frequency in Bandwidth (between f_low and f_high). If bandwidth is 250 kHz and carrier frequency is 433 MHz, the chirp goes between 432,875 MHz and 433,125 MHz.

### LNA Boost Gain

This parameter basically manages the gain Low Noise Amplifier, so affects amplification of received signal. If automatic gain control is set, reading 0x0C register returns current gain value.

### Spreading Factor

Impact of the Spreading Factor(Sweep Rate): higher value increases the packet's time in the air, extends the communication range, but also requires more energy to transmit data and reduces data transmission  (assuming the same Bandwidth).

Spreading Factor can be set from 6 to 12 (written into register 0x1E). Take a note (4.1.1.2. SX1278’s Datasheet) that setting SF6 needs Implicit Header Mode, and setting two other register properly (0x31 and 0x37).

### Bandwidth

Bandwidth determines the range in which the Chirp frequency changes from f_low to f_high. The more we increase it, the more frequency rise/fall slope (derivative) increases, which also increases the data transfer rate at the same Spreading Factor. On the other hand, too wide Bandwidth exposes you to interference from other devices and limits its  ability to transmit. Also, a narrower Bandwidth improves the transmission range.

### Coding rate

There are available modes: 4/5, 4/6, 4/7, 4/8. Coding Rate 4/5 means that from 5 bits of transmitted data, 4 are used functionally (filled with data), and rest are used for error correction. 4/8 is the smallest Coding Rate, giving the longest time to send the package and having highest resistance to interference. Changing the Coding Rate may increase the transmission quality, but reduces the data rate and potentially battery life.

### Implicit/Explicit Header Mode

Implicit or Explicit mode defined in register 0x1D determines if packet should cointain generated header with information about payload size, coding rate and payload CRC presence. In cases where all that settings are set and known on both sides, an implicit header mode could be set to reduce transmission time.

### CRC

Using register 0x1E you can define if payload CRC is enabled in payload. If CRC is needed in implicit mode, it should be enabled on both, receiver and transmitter side. In explicit mode the transmitter should have it CRC enabled. To check CRC errors, receiver should read the 0x1C register to determine if CRC was present in received packet and then check in 0x11 register CRC error flag.

### TX Power

Transmitting power should be set on transmitter side, but together with PA output pin setting, both in register 0x09. Output power formula usage depends on PA output pin choose: RFO pin or PA boost pin. Please be aware that depending on module, different output pin setting may be needed. Setting TX power correlates also with module current protection (register 0x0B).

### Notes

The Ra-01 has an attached 433 MHz antenna, so this determines the frequency use range of the module. In your environment, many devices may operate at this frequency, so try to adjust the parameters to your environment and check the legal regulations that apply to radio devices in your country.

If you have connectivity issues, focus on initially on your carrier frequency (a lot of wireless devices may work around your environment around frequency 433MHz) and your Bandwidth.
Combining a small Spreading Factor with a narrow bandwidth in the SX1278 module may require an external oscillator to increase sensitivity.
