################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libraries/HD44780_LCD/LCD_16x2_PARALLEL.c 

OBJS += \
./libraries/HD44780_LCD/LCD_16x2_PARALLEL.o 

C_DEPS += \
./libraries/HD44780_LCD/LCD_16x2_PARALLEL.d 


# Each subdirectory must supply rules for building sources it contributes
libraries/HD44780_LCD/%.o libraries/HD44780_LCD/%.su libraries/HD44780_LCD/%.cyclo: ../libraries/HD44780_LCD/%.c libraries/HD44780_LCD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"C:/Users/Linas/Desktop/STM32_stuff/Projects_older_workspace/Battery_analyzer/libraries/HD44780_LCD" -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-libraries-2f-HD44780_LCD

clean-libraries-2f-HD44780_LCD:
	-$(RM) ./libraries/HD44780_LCD/LCD_16x2_PARALLEL.cyclo ./libraries/HD44780_LCD/LCD_16x2_PARALLEL.d ./libraries/HD44780_LCD/LCD_16x2_PARALLEL.o ./libraries/HD44780_LCD/LCD_16x2_PARALLEL.su

.PHONY: clean-libraries-2f-HD44780_LCD

