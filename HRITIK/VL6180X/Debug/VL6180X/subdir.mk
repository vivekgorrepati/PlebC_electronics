################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../VL6180X/VL6180X.c 

OBJS += \
./VL6180X/VL6180X.o 

C_DEPS += \
./VL6180X/VL6180X.d 


# Each subdirectory must supply rules for building sources it contributes
VL6180X/%.o VL6180X/%.su VL6180X/%.cyclo: ../VL6180X/%.c VL6180X/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"E:/ST/Projects/VL6180X/VL6180X" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-VL6180X

clean-VL6180X:
	-$(RM) ./VL6180X/VL6180X.cyclo ./VL6180X/VL6180X.d ./VL6180X/VL6180X.o ./VL6180X/VL6180X.su

.PHONY: clean-VL6180X

