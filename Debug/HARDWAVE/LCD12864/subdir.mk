################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../HARDWAVE/LCD12864/LCD12864.c 

OBJS += \
./HARDWAVE/LCD12864/LCD12864.o 

C_DEPS += \
./HARDWAVE/LCD12864/LCD12864.d 


# Each subdirectory must supply rules for building sources it contributes
HARDWAVE/LCD12864/%.o HARDWAVE/LCD12864/%.su HARDWAVE/LCD12864/%.cyclo: ../HARDWAVE/LCD12864/%.c HARDWAVE/LCD12864/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/wolf.long/Documents/STM32_Project/Mpu6050And12864/HARDWAVE/MPU6050/Inc" -I"C:/Users/wolf.long/Documents/STM32_Project/Mpu6050And12864/HARDWAVE/MPU6050/eMPL" -I"C:/Users/wolf.long/Documents/STM32_Project/Mpu6050And12864/HARDWAVE/LCD12864" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-HARDWAVE-2f-LCD12864

clean-HARDWAVE-2f-LCD12864:
	-$(RM) ./HARDWAVE/LCD12864/LCD12864.cyclo ./HARDWAVE/LCD12864/LCD12864.d ./HARDWAVE/LCD12864/LCD12864.o ./HARDWAVE/LCD12864/LCD12864.su

.PHONY: clean-HARDWAVE-2f-LCD12864

