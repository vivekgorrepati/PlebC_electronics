################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Loadcell/loadcell.c 

OBJS += \
./Loadcell/loadcell.o 

C_DEPS += \
./Loadcell/loadcell.d 


# Each subdirectory must supply rules for building sources it contributes
Loadcell/%.o Loadcell/%.su Loadcell/%.cyclo: ../Loadcell/%.c Loadcell/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"E:/ST/Projects/Probe_Pad_Master_L476RG/BNO055" -I"E:/ST/Projects/Probe_Pad_Master_L476RG/Loadcell" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Loadcell

clean-Loadcell:
	-$(RM) ./Loadcell/loadcell.cyclo ./Loadcell/loadcell.d ./Loadcell/loadcell.o ./Loadcell/loadcell.su

.PHONY: clean-Loadcell

