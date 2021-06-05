################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/simon/Documentos/FACULTAD_linux/Microcontroladores/Tesis/firmware_log_sensortile_Multi/v1.0.0/Middlewares/ST/STM32_MetaDataManager/MetaDataManager.c 

OBJS += \
./Middlewares/STM32_MetaDataManager/MetaDataManager.o 

C_DEPS += \
./Middlewares/STM32_MetaDataManager/MetaDataManager.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/STM32_MetaDataManager/MetaDataManager.o: /home/simon/Documentos/FACULTAD_linux/Microcontroladores/Tesis/firmware_log_sensortile_Multi/v1.0.0/Middlewares/ST/STM32_MetaDataManager/MetaDataManager.c Middlewares/STM32_MetaDataManager/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g1 -DUSE_HAL_DRIVER -DOSX_BMS_SENSORTILE -DSTM32L476xx -DUSE_STM32L4XX_NUCLEO -c -I../../../Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../Drivers/BSP/Components/lsm6dsm -I../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../Drivers/BSP/Components/lsm303agr -I../../../../../../../Drivers/BSP/Components/stc3115 -I../../../../../../../Drivers/BSP/SensorTile -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../../../../../../Middlewares/Third_Party/FatFs/src -I../../../../../../../Middlewares/Third_Party/FatFs/src/drivers -I../../../../../../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I"../../../../../../../Drivers/BSP/Sensortile2" -I../../../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionFD_Library/Inc -I"../../../../../../../Middlewares/ST/STM32_MetaDataManager" -I../../../../../../../Middlewares/ST/STM32_MotionPW_Library/Inc -I../../../../../../../Middlewares/ST/STM32_MotionAW_Library/Inc -I../../../../../../../Middlewares/ST/BlueNRG-MS/includes -I../../../../../../../Middlewares/ST/BlueNRG-MS/utils -I../../../../../../../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -O0 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/STM32_MetaDataManager/MetaDataManager.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"

