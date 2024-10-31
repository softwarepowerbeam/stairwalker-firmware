################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32h723zgtx.s 

S_DEPS += \
./Core/Startup/startup_stm32h723zgtx.d 

OBJS += \
./Core/Startup/startup_stm32h723zgtx.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m7 -g3 -DDEBUG -c -I"C:/Users/Eduardo/STM32CubeIDE/workspace_wheelchair_udp/wheelchair/Application/Inc" -I"C:/Users/Eduardo/STM32CubeIDE/firmware_libs/PBLibs/BoardSupport/Inc" -I"C:/Users/Eduardo/STM32CubeIDE/firmware_libs/firmware-libs/Interfaces" -I"C:/Users/Eduardo/STM32CubeIDE/firmware_libs/firmware-libs/Middleware/Inc" -I"C:/Users/Eduardo/STM32CubeIDE/firmware_libs/firmware-libs/UserInterface/Inc" -I"C:/Users/Eduardo/STM32CubeIDE/workspace_wheelchair_udp/wheelchair/pb_lib/Inc" -I"C:/Users/Eduardo/STM32CubeIDE/firmware_libs/PBLibs/PB_HAL_STM32H7/Inc" -I"C:/Users/Eduardo/STM32CubeIDE/firmware_libs/PBLibs/UserInterface/Inc" -I"C:/Users/Eduardo/STM32CubeIDE/firmware_libs/firmware-libs/StatusCodes/Inc" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=softfp -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32h723zgtx.d ./Core/Startup/startup_stm32h723zgtx.o

.PHONY: clean-Core-2f-Startup

