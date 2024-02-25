################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../HARDWAVE/MPU6050/mpu6050iic.c 

OBJS += \
./HARDWAVE/MPU6050/mpu6050iic.o 

C_DEPS += \
./HARDWAVE/MPU6050/mpu6050iic.d 


# Each subdirectory must supply rules for building sources it contributes
HARDWAVE/MPU6050/%.o HARDWAVE/MPU6050/%.su HARDWAVE/MPU6050/%.cyclo: ../HARDWAVE/MPU6050/%.c HARDWAVE/MPU6050/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-HARDWAVE-2f-MPU6050

clean-HARDWAVE-2f-MPU6050:
	-$(RM) ./HARDWAVE/MPU6050/mpu6050iic.cyclo ./HARDWAVE/MPU6050/mpu6050iic.d ./HARDWAVE/MPU6050/mpu6050iic.o ./HARDWAVE/MPU6050/mpu6050iic.su

.PHONY: clean-HARDWAVE-2f-MPU6050

