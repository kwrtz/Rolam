################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/Documents/STM32/workspace/Schleifensensor/Src/calculation.c \
D:/Documents/STM32/workspace/Schleifensensor/Src/cmd.c \
D:/Documents/STM32/workspace/Schleifensensor/Src/led.c \
D:/Documents/STM32/workspace/Schleifensensor/Src/main.c \
D:/Documents/STM32/workspace/Schleifensensor/Src/send.c \
D:/Documents/STM32/workspace/Schleifensensor/Src/stm32f4xx_hal_msp.c \
D:/Documents/STM32/workspace/Schleifensensor/Src/stm32f4xx_it.c \
D:/Documents/STM32/workspace/Schleifensensor/Src/syscalls.c 

OBJS += \
./Application/User/calculation.o \
./Application/User/cmd.o \
./Application/User/led.o \
./Application/User/main.o \
./Application/User/send.o \
./Application/User/stm32f4xx_hal_msp.o \
./Application/User/stm32f4xx_it.o \
./Application/User/syscalls.o 

C_DEPS += \
./Application/User/calculation.d \
./Application/User/cmd.d \
./Application/User/led.d \
./Application/User/main.d \
./Application/User/send.d \
./Application/User/stm32f4xx_hal_msp.d \
./Application/User/stm32f4xx_it.d \
./Application/User/syscalls.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/calculation.o: D:/Documents/STM32/workspace/Schleifensensor/Src/calculation.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F446xx -I"D:/Documents/STM32/workspace/Schleifensensor/Inc" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/CMSIS/Include" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/Documents/STM32/workspace/Schleifensensor/Inc"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/cmd.o: D:/Documents/STM32/workspace/Schleifensensor/Src/cmd.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F446xx -I"D:/Documents/STM32/workspace/Schleifensensor/Inc" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/CMSIS/Include" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/Documents/STM32/workspace/Schleifensensor/Inc"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/led.o: D:/Documents/STM32/workspace/Schleifensensor/Src/led.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F446xx -I"D:/Documents/STM32/workspace/Schleifensensor/Inc" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/CMSIS/Include" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/Documents/STM32/workspace/Schleifensensor/Inc"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/main.o: D:/Documents/STM32/workspace/Schleifensensor/Src/main.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F446xx -I"D:/Documents/STM32/workspace/Schleifensensor/Inc" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/CMSIS/Include" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/Documents/STM32/workspace/Schleifensensor/Inc"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/send.o: D:/Documents/STM32/workspace/Schleifensensor/Src/send.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F446xx -I"D:/Documents/STM32/workspace/Schleifensensor/Inc" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/CMSIS/Include" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/Documents/STM32/workspace/Schleifensensor/Inc"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/stm32f4xx_hal_msp.o: D:/Documents/STM32/workspace/Schleifensensor/Src/stm32f4xx_hal_msp.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F446xx -I"D:/Documents/STM32/workspace/Schleifensensor/Inc" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/CMSIS/Include" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/Documents/STM32/workspace/Schleifensensor/Inc"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/stm32f4xx_it.o: D:/Documents/STM32/workspace/Schleifensensor/Src/stm32f4xx_it.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F446xx -I"D:/Documents/STM32/workspace/Schleifensensor/Inc" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/CMSIS/Include" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/Documents/STM32/workspace/Schleifensensor/Inc"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/syscalls.o: D:/Documents/STM32/workspace/Schleifensensor/Src/syscalls.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F446xx -I"D:/Documents/STM32/workspace/Schleifensensor/Inc" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/CMSIS/Include" -I"D:/Documents/STM32/workspace/Schleifensensor/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/Documents/STM32/workspace/Schleifensensor/Inc"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


