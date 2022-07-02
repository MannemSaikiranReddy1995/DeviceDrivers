################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/stm32f407xx_gpio_driver.c 

OBJS += \
./Drivers/Src/stm32f407xx_gpio_driver.o 

C_DEPS += \
./Drivers/Src/stm32f407xx_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"C:/SaikiranReddyMannem/RTOS_WorkSpace/DeviceDriverProjects/STM32F407_DISC_Drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/stm32f407xx_gpio_driver.d ./Drivers/Src/stm32f407xx_gpio_driver.o

.PHONY: clean-Drivers-2f-Src

