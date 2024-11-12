################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../PlebcStepper/PlebcStepper.c 

OBJS += \
./PlebcStepper/PlebcStepper.o 

C_DEPS += \
./PlebcStepper/PlebcStepper.d 


# Each subdirectory must supply rules for building sources it contributes
PlebcStepper/%.o PlebcStepper/%.su PlebcStepper/%.cyclo: ../PlebcStepper/%.c PlebcStepper/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"E:/ST/Projects/Code for M2 Controller/MODBUS" -I"E:/ST/Projects/Code for M2 Controller/PlebcStepper" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-PlebcStepper

clean-PlebcStepper:
	-$(RM) ./PlebcStepper/PlebcStepper.cyclo ./PlebcStepper/PlebcStepper.d ./PlebcStepper/PlebcStepper.o ./PlebcStepper/PlebcStepper.su

.PHONY: clean-PlebcStepper

