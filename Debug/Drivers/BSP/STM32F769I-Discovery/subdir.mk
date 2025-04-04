################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery.c 

OBJS += \
./Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery.o 

C_DEPS += \
./Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM32F769I-Discovery/%.o Drivers/BSP/STM32F769I-Discovery/%.su Drivers/BSP/STM32F769I-Discovery/%.cyclo: ../Drivers/BSP/STM32F769I-Discovery/%.c Drivers/BSP/STM32F769I-Discovery/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F769xx -c -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/adamc/STM32CubeIDE/pracownia/LACZNIK/Drivers/BSP/Components" -I"C:/Users/adamc/STM32CubeIDE/pracownia/LACZNIK/Drivers/BSP/STM32F769I-Discovery" -I"C:/Users/adamc/STM32CubeIDE/pracownia/LACZNIK/Drivers/BSP/Components/Common" -I"C:/Users/adamc/STM32CubeIDE/pracownia/LACZNIK/Drivers/BSP/Components/wm8994" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-STM32F769I-2d-Discovery

clean-Drivers-2f-BSP-2f-STM32F769I-2d-Discovery:
	-$(RM) ./Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery.cyclo ./Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery.d ./Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery.o ./Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery.su

.PHONY: clean-Drivers-2f-BSP-2f-STM32F769I-2d-Discovery

