################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/DD_MotorControl.c \
../Core/Src/ReceiveModule.c \
../Core/Src/TransmitModule.c \
../Core/Src/UltraS_Motors.c \
../Core/Src/UltrasonicSensors.c \
../Core/Src/adc.c \
../Core/Src/battery.c \
../Core/Src/callbacks.c \
../Core/Src/camera.c \
../Core/Src/ccs811_async.c \
../Core/Src/dcmi.c \
../Core/Src/dma.c \
../Core/Src/esp_driver.c \
../Core/Src/freertos.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/lcd.c \
../Core/Src/main.c \
../Core/Src/movement_control.c \
../Core/Src/sensor1081_async.c \
../Core/Src/spi.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_hal_timebase_tim.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/DD_MotorControl.o \
./Core/Src/ReceiveModule.o \
./Core/Src/TransmitModule.o \
./Core/Src/UltraS_Motors.o \
./Core/Src/UltrasonicSensors.o \
./Core/Src/adc.o \
./Core/Src/battery.o \
./Core/Src/callbacks.o \
./Core/Src/camera.o \
./Core/Src/ccs811_async.o \
./Core/Src/dcmi.o \
./Core/Src/dma.o \
./Core/Src/esp_driver.o \
./Core/Src/freertos.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/lcd.o \
./Core/Src/main.o \
./Core/Src/movement_control.o \
./Core/Src/sensor1081_async.o \
./Core/Src/spi.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_hal_timebase_tim.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/DD_MotorControl.d \
./Core/Src/ReceiveModule.d \
./Core/Src/TransmitModule.d \
./Core/Src/UltraS_Motors.d \
./Core/Src/UltrasonicSensors.d \
./Core/Src/adc.d \
./Core/Src/battery.d \
./Core/Src/callbacks.d \
./Core/Src/camera.d \
./Core/Src/ccs811_async.d \
./Core/Src/dcmi.d \
./Core/Src/dma.d \
./Core/Src/esp_driver.d \
./Core/Src/freertos.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/lcd.d \
./Core/Src/main.d \
./Core/Src/movement_control.d \
./Core/Src/sensor1081_async.d \
./Core/Src/spi.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_hal_timebase_tim.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/DD_MotorControl.cyclo ./Core/Src/DD_MotorControl.d ./Core/Src/DD_MotorControl.o ./Core/Src/DD_MotorControl.su ./Core/Src/ReceiveModule.cyclo ./Core/Src/ReceiveModule.d ./Core/Src/ReceiveModule.o ./Core/Src/ReceiveModule.su ./Core/Src/TransmitModule.cyclo ./Core/Src/TransmitModule.d ./Core/Src/TransmitModule.o ./Core/Src/TransmitModule.su ./Core/Src/UltraS_Motors.cyclo ./Core/Src/UltraS_Motors.d ./Core/Src/UltraS_Motors.o ./Core/Src/UltraS_Motors.su ./Core/Src/UltrasonicSensors.cyclo ./Core/Src/UltrasonicSensors.d ./Core/Src/UltrasonicSensors.o ./Core/Src/UltrasonicSensors.su ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/battery.cyclo ./Core/Src/battery.d ./Core/Src/battery.o ./Core/Src/battery.su ./Core/Src/callbacks.cyclo ./Core/Src/callbacks.d ./Core/Src/callbacks.o ./Core/Src/callbacks.su ./Core/Src/camera.cyclo ./Core/Src/camera.d ./Core/Src/camera.o ./Core/Src/camera.su ./Core/Src/ccs811_async.cyclo ./Core/Src/ccs811_async.d ./Core/Src/ccs811_async.o ./Core/Src/ccs811_async.su ./Core/Src/dcmi.cyclo ./Core/Src/dcmi.d ./Core/Src/dcmi.o ./Core/Src/dcmi.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/esp_driver.cyclo ./Core/Src/esp_driver.d ./Core/Src/esp_driver.o ./Core/Src/esp_driver.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/lcd.cyclo ./Core/Src/lcd.d ./Core/Src/lcd.o ./Core/Src/lcd.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/movement_control.cyclo ./Core/Src/movement_control.d ./Core/Src/movement_control.o ./Core/Src/movement_control.su ./Core/Src/sensor1081_async.cyclo ./Core/Src/sensor1081_async.d ./Core/Src/sensor1081_async.o ./Core/Src/sensor1081_async.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_hal_timebase_tim.cyclo ./Core/Src/stm32h7xx_hal_timebase_tim.d ./Core/Src/stm32h7xx_hal_timebase_tim.o ./Core/Src/stm32h7xx_hal_timebase_tim.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

