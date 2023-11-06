################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include sx127x/src/subdir.mk
-include startup/subdir.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: STM32F1_ra01_receiver.elf

dependents:
	-cd C:/Users/M/workspace/stm32f103c8t6_board_hal_lib/Debug && $(MAKE) all

# Tool invocations
STM32F1_ra01_receiver.elf: $(OBJS) $(USER_OBJS) C:/Users/M/workspace/stm32f103c8t6_board_hal_lib/Debug/libstm32f103c8t6_board_hal_lib.a C:\Users\M\workspace\STM32F1_ra01_receiver/LinkerScript.ld
	@echo 'Building target: $@'
	@echo 'Invoking: MCU GCC Linker'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -L"C:\Users\M\workspace\stm32f103c8t6_board_hal_lib\Debug" -T"C:/Users/M/workspace/STM32F1_ra01_receiver/LinkerScript.ld" -Wl,-Map=output.map -Wl,--gc-sections -o "STM32F1_ra01_receiver.elf" @"objects.list" $(USER_OBJS) $(LIBS) -lm
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

post-build:
	-@echo 'Generating binary and Printing size information:'
	arm-none-eabi-objcopy -O binary "STM32F1_ra01_receiver.elf" "STM32F1_ra01_receiver.bin"
	arm-none-eabi-size "STM32F1_ra01_receiver.elf"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build
C:/Users/M/workspace/stm32f103c8t6_board_hal_lib/Debug/libstm32f103c8t6_board_hal_lib.a:

-include ../makefile.targets
