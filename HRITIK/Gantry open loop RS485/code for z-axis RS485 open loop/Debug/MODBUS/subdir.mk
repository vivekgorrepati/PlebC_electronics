################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MODBUS/modbus_rtu_slave.c 

OBJS += \
./MODBUS/modbus_rtu_slave.o 

C_DEPS += \
./MODBUS/modbus_rtu_slave.d 


# Each subdirectory must supply rules for building sources it contributes
MODBUS/%.o MODBUS/%.su MODBUS/%.cyclo: ../MODBUS/%.c MODBUS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"E:/ST/Projects/code for z-axis RS485 open loop/MODBUS" -I"E:/ST/Projects/code for z-axis RS485 open loop/PlebcStepper" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-MODBUS

clean-MODBUS:
	-$(RM) ./MODBUS/modbus_rtu_slave.cyclo ./MODBUS/modbus_rtu_slave.d ./MODBUS/modbus_rtu_slave.o ./MODBUS/modbus_rtu_slave.su

.PHONY: clean-MODBUS

