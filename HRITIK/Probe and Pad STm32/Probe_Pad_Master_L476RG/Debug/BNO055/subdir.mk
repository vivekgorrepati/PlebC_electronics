################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BNO055/bno055.c 

OBJS += \
./BNO055/bno055.o 

C_DEPS += \
./BNO055/bno055.d 


# Each subdirectory must supply rules for building sources it contributes
BNO055/%.o BNO055/%.su BNO055/%.cyclo: ../BNO055/%.c BNO055/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"E:/ST/Projects/Probe_Pad_Master_L476RG/BNO055" -I"E:/ST/Projects/Probe_Pad_Master_L476RG/Loadcell" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-BNO055

clean-BNO055:
	-$(RM) ./BNO055/bno055.cyclo ./BNO055/bno055.d ./BNO055/bno055.o ./BNO055/bno055.su

.PHONY: clean-BNO055

