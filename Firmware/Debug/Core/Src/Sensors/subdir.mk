################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/Sensors/BMP280.cpp \
../Core/Src/Sensors/ICM42670P.cpp 

OBJS += \
./Core/Src/Sensors/BMP280.o \
./Core/Src/Sensors/ICM42670P.o 

CPP_DEPS += \
./Core/Src/Sensors/BMP280.d \
./Core/Src/Sensors/ICM42670P.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Sensors/%.o Core/Src/Sensors/%.su Core/Src/Sensors/%.cyclo: ../Core/Src/Sensors/%.cpp Core/Src/Sensors/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Sensors

clean-Core-2f-Src-2f-Sensors:
	-$(RM) ./Core/Src/Sensors/BMP280.cyclo ./Core/Src/Sensors/BMP280.d ./Core/Src/Sensors/BMP280.o ./Core/Src/Sensors/BMP280.su ./Core/Src/Sensors/ICM42670P.cyclo ./Core/Src/Sensors/ICM42670P.d ./Core/Src/Sensors/ICM42670P.o ./Core/Src/Sensors/ICM42670P.su

.PHONY: clean-Core-2f-Src-2f-Sensors

