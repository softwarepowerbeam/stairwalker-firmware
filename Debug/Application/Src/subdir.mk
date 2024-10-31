################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Application/Src/cpp_link.cpp 

OBJS += \
./Application/Src/cpp_link.o 

CPP_DEPS += \
./Application/Src/cpp_link.d 


# Each subdirectory must supply rules for building sources it contributes
Application/Src/%.o Application/Src/%.su Application/Src/%.cyclo: ../Application/Src/%.cpp Application/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/Eduardo/STM32CubeIDE/workspace_wheelchair_udp/wheelchair/Application/Inc" -I"C:/Users/Eduardo/STM32CubeIDE/firmware_libs/PBLibs/BoardSupport/Inc" -I"C:/Users/Eduardo/STM32CubeIDE/firmware_libs/firmware-libs/Interfaces" -I"C:/Users/Eduardo/STM32CubeIDE/firmware_libs/firmware-libs/Middleware/Inc" -I"C:/Users/Eduardo/STM32CubeIDE/firmware_libs/firmware-libs/UserInterface/Inc" -I"C:/Users/Eduardo/STM32CubeIDE/workspace_wheelchair_udp/wheelchair/pb_lib/Inc" -I"C:/Users/Eduardo/STM32CubeIDE/firmware_libs/PBLibs/PB_HAL_STM32H7/Inc" -I"C:/Users/Eduardo/STM32CubeIDE/firmware_libs/PBLibs/UserInterface/Inc" -I"C:/Users/Eduardo/STM32CubeIDE/firmware_libs/firmware-libs/StatusCodes/Inc" -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Drivers/BSP/Components/lan8742 -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -I"C:/Users/Eduardo/STM32CubeIDE/firmware_libs/firmware-libs/BoardSupport/Inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=softfp -mthumb -o "$@"

clean: clean-Application-2f-Src

clean-Application-2f-Src:
	-$(RM) ./Application/Src/cpp_link.cyclo ./Application/Src/cpp_link.d ./Application/Src/cpp_link.o ./Application/Src/cpp_link.su

.PHONY: clean-Application-2f-Src

