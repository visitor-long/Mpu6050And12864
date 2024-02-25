################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../HARDWAVE/MPU6050/eMPL/inv_mpu.c \
../HARDWAVE/MPU6050/eMPL/inv_mpu_dmp_motion_driver.c 

OBJS += \
./HARDWAVE/MPU6050/eMPL/inv_mpu.o \
./HARDWAVE/MPU6050/eMPL/inv_mpu_dmp_motion_driver.o 

C_DEPS += \
./HARDWAVE/MPU6050/eMPL/inv_mpu.d \
./HARDWAVE/MPU6050/eMPL/inv_mpu_dmp_motion_driver.d 


# Each subdirectory must supply rules for building sources it contributes
HARDWAVE/MPU6050/eMPL/%.o HARDWAVE/MPU6050/eMPL/%.su HARDWAVE/MPU6050/eMPL/%.cyclo: ../HARDWAVE/MPU6050/eMPL/%.c HARDWAVE/MPU6050/eMPL/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/wolf.long/Documents/STM32_Project/Mpu6050And12864/HARDWAVE/MPU6050/Inc" -I"C:/Users/wolf.long/Documents/STM32_Project/Mpu6050And12864/HARDWAVE/MPU6050/eMPL" -I"C:/Users/wolf.long/Documents/STM32_Project/Mpu6050And12864/HARDWAVE/LCD12864" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-HARDWAVE-2f-MPU6050-2f-eMPL

clean-HARDWAVE-2f-MPU6050-2f-eMPL:
	-$(RM) ./HARDWAVE/MPU6050/eMPL/inv_mpu.cyclo ./HARDWAVE/MPU6050/eMPL/inv_mpu.d ./HARDWAVE/MPU6050/eMPL/inv_mpu.o ./HARDWAVE/MPU6050/eMPL/inv_mpu.su ./HARDWAVE/MPU6050/eMPL/inv_mpu_dmp_motion_driver.cyclo ./HARDWAVE/MPU6050/eMPL/inv_mpu_dmp_motion_driver.d ./HARDWAVE/MPU6050/eMPL/inv_mpu_dmp_motion_driver.o ./HARDWAVE/MPU6050/eMPL/inv_mpu_dmp_motion_driver.su

.PHONY: clean-HARDWAVE-2f-MPU6050-2f-eMPL

