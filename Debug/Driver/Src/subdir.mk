################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Driver/Src/stm32f767xx_gpio_driver.c 

OBJS += \
./Driver/Src/stm32f767xx_gpio_driver.o 

C_DEPS += \
./Driver/Src/stm32f767xx_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Driver/Src/%.o Driver/Src/%.su: ../Driver/Src/%.c Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DSTM32 -DSTM32F7 -DSTM32F767ZITx -c -I../Inc -I"C:/Code/stm32f7xx_drivers/Driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Driver-2f-Src

clean-Driver-2f-Src:
	-$(RM) ./Driver/Src/stm32f767xx_gpio_driver.d ./Driver/Src/stm32f767xx_gpio_driver.o ./Driver/Src/stm32f767xx_gpio_driver.su

.PHONY: clean-Driver-2f-Src

