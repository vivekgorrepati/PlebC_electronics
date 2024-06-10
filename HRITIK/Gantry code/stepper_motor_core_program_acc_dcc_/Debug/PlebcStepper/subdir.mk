################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
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
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"C:/Users/AKASH BEHRA/STM32CubeIDE/workspace_1.12.1/stepper_motor_core_program_acc_dcc_2/PlebcStepper" -I"C:/Users/AKASH BEHRA/STM32CubeIDE/workspace_1.12.1/stepper_motor_core_program_acc_dcc_2/Drivers/STM32F4xx_HAL_Driver/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/AKASH BEHRA/STM32CubeIDE/workspace_1.12.1/stepper_motor_core_program_acc_dcc_2/PlebcStepper" -I"C:/Users/AKASH BEHRA/STM32CubeIDE/workspace_1.12.1/stepper_motor_core_program_acc_dcc_2/Drivers/STM32F4xx_HAL_Driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-PlebcStepper

clean-PlebcStepper:
	-$(RM) ./PlebcStepper/PlebcStepper.cyclo ./PlebcStepper/PlebcStepper.d ./PlebcStepper/PlebcStepper.o ./PlebcStepper/PlebcStepper.su

.PHONY: clean-PlebcStepper

