################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../startup/startup_stm32.s 

OBJS += \
./startup/startup_stm32.o 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo $(PWD)
	arm-none-eabi-as -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -I"C:/Users/M/workspace/stm32f103c8t6_board_hal_lib" -I"C:/Users/M/workspace/stm32f103c8t6_board_hal_lib/CMSIS/core" -I"C:/Users/M/workspace/stm32f103c8t6_board_hal_lib/CMSIS/device" -I"C:/Users/M/workspace/stm32f103c8t6_board_hal_lib/HAL_Driver/Inc/Legacy" -I"C:/Users/M/workspace/stm32f103c8t6_board_hal_lib/HAL_Driver/Inc" -I"C:/Users/M/workspace/STM32F1_ra01_sender/inc" -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


