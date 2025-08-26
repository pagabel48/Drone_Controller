################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/Control/altitude_kf.cpp \
../Core/Src/Control/attitude_kf.cpp \
../Core/Src/Control/attitude_pid.cpp \
../Core/Src/Control/controller.cpp \
../Core/Src/Control/pid.cpp 

OBJS += \
./Core/Src/Control/altitude_kf.o \
./Core/Src/Control/attitude_kf.o \
./Core/Src/Control/attitude_pid.o \
./Core/Src/Control/controller.o \
./Core/Src/Control/pid.o 

CPP_DEPS += \
./Core/Src/Control/altitude_kf.d \
./Core/Src/Control/attitude_kf.d \
./Core/Src/Control/attitude_pid.d \
./Core/Src/Control/controller.d \
./Core/Src/Control/pid.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Control/%.o Core/Src/Control/%.su Core/Src/Control/%.cyclo: ../Core/Src/Control/%.cpp Core/Src/Control/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Control

clean-Core-2f-Src-2f-Control:
	-$(RM) ./Core/Src/Control/altitude_kf.cyclo ./Core/Src/Control/altitude_kf.d ./Core/Src/Control/altitude_kf.o ./Core/Src/Control/altitude_kf.su ./Core/Src/Control/attitude_kf.cyclo ./Core/Src/Control/attitude_kf.d ./Core/Src/Control/attitude_kf.o ./Core/Src/Control/attitude_kf.su ./Core/Src/Control/attitude_pid.cyclo ./Core/Src/Control/attitude_pid.d ./Core/Src/Control/attitude_pid.o ./Core/Src/Control/attitude_pid.su ./Core/Src/Control/controller.cyclo ./Core/Src/Control/controller.d ./Core/Src/Control/controller.o ./Core/Src/Control/controller.su ./Core/Src/Control/pid.cyclo ./Core/Src/Control/pid.d ./Core/Src/Control/pid.o ./Core/Src/Control/pid.su

.PHONY: clean-Core-2f-Src-2f-Control

