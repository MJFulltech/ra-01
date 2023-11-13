################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/M/workspace/sx127x/src/sx127x.c \
C:/Users/M/workspace/sx127x/src/sx127x_stm32f1.c 

OBJS += \
./sx127x/src/sx127x.o \
./sx127x/src/sx127x_stm32f1.o 

C_DEPS += \
./sx127x/src/sx127x.d \
./sx127x/src/sx127x_stm32f1.d 


# Each subdirectory must supply rules for building sources it contributes
sx127x/src/sx127x.o: C:/Users/M/workspace/sx127x/src/sx127x.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -DSTM32F103xB -DUSE_HAL_DRIVER -I"C:/Users/M/workspace/stm32f103c8t6_board_hal_lib" -I"C:/Users/M/workspace/sx127x/inc" -I"C:/Users/M/workspace/stm32f103c8t6_board_hal_lib/CMSIS/core" -I"C:/Users/M/workspace/stm32f103c8t6_board_hal_lib/CMSIS/device" -I"C:/Users/M/workspace/stm32f103c8t6_board_hal_lib/HAL_Driver/Inc/Legacy" -I"C:/Users/M/workspace/stm32f103c8t6_board_hal_lib/HAL_Driver/Inc" -I"C:/Users/M/workspace/STM32F1_ra01_receiver_irq/inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

sx127x/src/sx127x_stm32f1.o: C:/Users/M/workspace/sx127x/src/sx127x_stm32f1.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -DSTM32F103xB -DUSE_HAL_DRIVER -I"C:/Users/M/workspace/stm32f103c8t6_board_hal_lib" -I"C:/Users/M/workspace/sx127x/inc" -I"C:/Users/M/workspace/stm32f103c8t6_board_hal_lib/CMSIS/core" -I"C:/Users/M/workspace/stm32f103c8t6_board_hal_lib/CMSIS/device" -I"C:/Users/M/workspace/stm32f103c8t6_board_hal_lib/HAL_Driver/Inc/Legacy" -I"C:/Users/M/workspace/stm32f103c8t6_board_hal_lib/HAL_Driver/Inc" -I"C:/Users/M/workspace/STM32F1_ra01_receiver_irq/inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


