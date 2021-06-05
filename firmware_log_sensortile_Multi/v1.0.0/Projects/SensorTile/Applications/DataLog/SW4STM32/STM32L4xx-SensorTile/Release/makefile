################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/STM32_USBD_Library/Core/subdir.mk
-include Middlewares/STM32_USBD_Library/Class/subdir.mk
-include Middlewares/STM32_MetaDataManager/subdir.mk
-include Middlewares/FatFs/subdir.mk
-include Middlewares/BlueNRG-MS/utils/subdir.mk
-include Middlewares/BlueNRG-MS/hci/hci_tl_patterns/Basic/subdir.mk
-include Middlewares/BlueNRG-MS/hci/controller/subdir.mk
-include Middlewares/BlueNRG-MS/subdir.mk
-include Drivers/STM32L4xx_HAL_Driver/subdir.mk
-include Drivers/CMSIS/subdir.mk
-include Drivers/BSP/Sensortile2/subdir.mk
-include Drivers/BSP/SensorTile/subdir.mk
-include Drivers/BSP/Components/subdir.mk
-include Doc/subdir.mk
-include DataLog/User/subdir.mk
-include DataLog/SW4STM32/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := Log_Bersa_Algo_Disparo
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX := 
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME).$(BUILD_ARTIFACT_EXTENSION)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Log_Bersa_Algo_Disparo.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Log_Bersa_Algo_Disparo.list \

OBJCOPY_BIN += \
Log_Bersa_Algo_Disparo.bin \


# All Target
all:
	+@$(MAKE) --no-print-directory main-build && $(MAKE) --no-print-directory post-build

# Main-build Target
main-build: Log_Bersa_Algo_Disparo.elf secondary-outputs

# Tool invocations
Log_Bersa_Algo_Disparo.elf: $(OBJS) $(USER_OBJS) ../LinkerScript.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Log_Bersa_Algo_Disparo.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"../LinkerScript.ld" -Wl,-Map="Log_Bersa_Algo_Disparo.map" -Wl,--gc-sections -static -Wl,--start-group -l:MotionFX_CM4F_wc32_ot.a -l:MotionFD_CM4F_wc32_ot.a -l:MotionPW_CM4F_wc32_ot.a -l:MotionAW_CM4F_wc32_ot.a -Wl,--end-group -L../../../../../../../Middlewares/ST/STM32_MotionFX_Library/Lib -L../../../../../../../Middlewares/ST/STM32_MotionFD_Library/Lib -L../../../../../../../Middlewares/ST/STM32_MotionPW_Library/Lib -L../../../../../../../Middlewares/ST/STM32_MotionAW_Library/Lib --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Log_Bersa_Algo_Disparo.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Log_Bersa_Algo_Disparo.list"
	@echo 'Finished building: $@'
	@echo ' '

Log_Bersa_Algo_Disparo.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Log_Bersa_Algo_Disparo.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

post-build:
	arm-none-eabi-objcopy -O binary "Log_Bersa_Algo_Disparo.elf" "Log_Bersa_Algo_Disparo.bin"
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents fail-specified-linker-script-missing warn-no-linker-script-specified
.SECONDARY: post-build

-include ../makefile.targets
