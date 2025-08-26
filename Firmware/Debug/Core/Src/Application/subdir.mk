################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/Application/application.cpp \
../Core/Src/Application/motors.cpp \
../Core/Src/Application/receiver.cpp \
../Core/Src/Application/test.cpp 

OBJS += \
./Core/Src/Application/application.o \
./Core/Src/Application/motors.o \
./Core/Src/Application/receiver.o \
./Core/Src/Application/test.o 

CPP_DEPS += \
./Core/Src/Application/application.d \
./Core/Src/Application/motors.d \
./Core/Src/Application/receiver.d \
./Core/Src/Application/test.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Application/%.o Core/Src/Application/%.su Core/Src/Application/%.cyclo: ../Core/Src/Application/%.cpp Core/Src/Application/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Application

clean-Core-2f-Src-2f-Application:
	-$(RM) ./Core/Src/Application/application.cyclo ./Core/Src/Application/application.d ./Core/Src/Application/application.o ./Core/Src/Application/application.su ./Core/Src/Application/motors.cyclo ./Core/Src/Application/motors.d ./Core/Src/Application/motors.o ./Core/Src/Application/motors.su ./Core/Src/Application/receiver.cyclo ./Core/Src/Application/receiver.d ./Core/Src/Application/receiver.o ./Core/Src/Application/receiver.su ./Core/Src/Application/test.cyclo ./Core/Src/Application/test.d ./Core/Src/Application/test.o ./Core/Src/Application/test.su

.PHONY: clean-Core-2f-Src-2f-Application

