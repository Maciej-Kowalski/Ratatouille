################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/BMI270_SensorAPI-master/STM32BMI270.c \
../Core/Inc/BMI270_SensorAPI-master/bmi2.c \
../Core/Inc/BMI270_SensorAPI-master/bmi270.c \
../Core/Inc/BMI270_SensorAPI-master/bmi270_context.c \
../Core/Inc/BMI270_SensorAPI-master/bmi270_legacy.c \
../Core/Inc/BMI270_SensorAPI-master/bmi270_maximum_fifo.c \
../Core/Inc/BMI270_SensorAPI-master/bmi2_ois.c \
../Core/Inc/BMI270_SensorAPI-master/common.c 

OBJS += \
./Core/Inc/BMI270_SensorAPI-master/STM32BMI270.o \
./Core/Inc/BMI270_SensorAPI-master/bmi2.o \
./Core/Inc/BMI270_SensorAPI-master/bmi270.o \
./Core/Inc/BMI270_SensorAPI-master/bmi270_context.o \
./Core/Inc/BMI270_SensorAPI-master/bmi270_legacy.o \
./Core/Inc/BMI270_SensorAPI-master/bmi270_maximum_fifo.o \
./Core/Inc/BMI270_SensorAPI-master/bmi2_ois.o \
./Core/Inc/BMI270_SensorAPI-master/common.o 

C_DEPS += \
./Core/Inc/BMI270_SensorAPI-master/STM32BMI270.d \
./Core/Inc/BMI270_SensorAPI-master/bmi2.d \
./Core/Inc/BMI270_SensorAPI-master/bmi270.d \
./Core/Inc/BMI270_SensorAPI-master/bmi270_context.d \
./Core/Inc/BMI270_SensorAPI-master/bmi270_legacy.d \
./Core/Inc/BMI270_SensorAPI-master/bmi270_maximum_fifo.d \
./Core/Inc/BMI270_SensorAPI-master/bmi2_ois.d \
./Core/Inc/BMI270_SensorAPI-master/common.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/BMI270_SensorAPI-master/%.o Core/Inc/BMI270_SensorAPI-master/%.su Core/Inc/BMI270_SensorAPI-master/%.cyclo: ../Core/Inc/BMI270_SensorAPI-master/%.c Core/Inc/BMI270_SensorAPI-master/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_NUCLEO_64 -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/P-NUCLEO-WB55.Nucleo -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-BMI270_SensorAPI-2d-master

clean-Core-2f-Inc-2f-BMI270_SensorAPI-2d-master:
	-$(RM) ./Core/Inc/BMI270_SensorAPI-master/STM32BMI270.cyclo ./Core/Inc/BMI270_SensorAPI-master/STM32BMI270.d ./Core/Inc/BMI270_SensorAPI-master/STM32BMI270.o ./Core/Inc/BMI270_SensorAPI-master/STM32BMI270.su ./Core/Inc/BMI270_SensorAPI-master/bmi2.cyclo ./Core/Inc/BMI270_SensorAPI-master/bmi2.d ./Core/Inc/BMI270_SensorAPI-master/bmi2.o ./Core/Inc/BMI270_SensorAPI-master/bmi2.su ./Core/Inc/BMI270_SensorAPI-master/bmi270.cyclo ./Core/Inc/BMI270_SensorAPI-master/bmi270.d ./Core/Inc/BMI270_SensorAPI-master/bmi270.o ./Core/Inc/BMI270_SensorAPI-master/bmi270.su ./Core/Inc/BMI270_SensorAPI-master/bmi270_context.cyclo ./Core/Inc/BMI270_SensorAPI-master/bmi270_context.d ./Core/Inc/BMI270_SensorAPI-master/bmi270_context.o ./Core/Inc/BMI270_SensorAPI-master/bmi270_context.su ./Core/Inc/BMI270_SensorAPI-master/bmi270_legacy.cyclo ./Core/Inc/BMI270_SensorAPI-master/bmi270_legacy.d ./Core/Inc/BMI270_SensorAPI-master/bmi270_legacy.o ./Core/Inc/BMI270_SensorAPI-master/bmi270_legacy.su ./Core/Inc/BMI270_SensorAPI-master/bmi270_maximum_fifo.cyclo ./Core/Inc/BMI270_SensorAPI-master/bmi270_maximum_fifo.d ./Core/Inc/BMI270_SensorAPI-master/bmi270_maximum_fifo.o ./Core/Inc/BMI270_SensorAPI-master/bmi270_maximum_fifo.su ./Core/Inc/BMI270_SensorAPI-master/bmi2_ois.cyclo ./Core/Inc/BMI270_SensorAPI-master/bmi2_ois.d ./Core/Inc/BMI270_SensorAPI-master/bmi2_ois.o ./Core/Inc/BMI270_SensorAPI-master/bmi2_ois.su ./Core/Inc/BMI270_SensorAPI-master/common.cyclo ./Core/Inc/BMI270_SensorAPI-master/common.d ./Core/Inc/BMI270_SensorAPI-master/common.o ./Core/Inc/BMI270_SensorAPI-master/common.su

.PHONY: clean-Core-2f-Inc-2f-BMI270_SensorAPI-2d-master

