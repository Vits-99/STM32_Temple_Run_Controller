################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/lis2dw12/lis2dw12.c \
../Drivers/BSP/Components/lis2dw12/lis2dw12_reg.c 

OBJS += \
./Drivers/BSP/Components/lis2dw12/lis2dw12.o \
./Drivers/BSP/Components/lis2dw12/lis2dw12_reg.o 

C_DEPS += \
./Drivers/BSP/Components/lis2dw12/lis2dw12.d \
./Drivers/BSP/Components/lis2dw12/lis2dw12_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/lis2dw12/%.o: ../Drivers/BSP/Components/lis2dw12/%.c Drivers/BSP/Components/lis2dw12/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../X-CUBE-MEMS1/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BSP/Components/lsm6dso -I../Drivers/BSP/Components/lis2dw12 -I../Drivers/BSP/Components/lis2mdl -I../Drivers/BSP/Components/hts221 -I../Drivers/BSP/Components/lps22hh -I../Drivers/BSP/Components/stts751 -I../Drivers/BSP/IKS01A3 -I../Drivers/BSP/Components/Common -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

