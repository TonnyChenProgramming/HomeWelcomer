################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/HC_SR04/hc_sr04.c 

OBJS += \
./Core/Src/HC_SR04/hc_sr04.o 

C_DEPS += \
./Core/Src/HC_SR04/hc_sr04.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/HC_SR04/%.o Core/Src/HC_SR04/%.su Core/Src/HC_SR04/%.cyclo: ../Core/Src/HC_SR04/%.c Core/Src/HC_SR04/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"../Core/Inc/display" -I"../Core/Src/display" -I"../Core/Src/HC_SR04" -I"../Core/Inc/HC_SR04" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-HC_SR04

clean-Core-2f-Src-2f-HC_SR04:
	-$(RM) ./Core/Src/HC_SR04/hc_sr04.cyclo ./Core/Src/HC_SR04/hc_sr04.d ./Core/Src/HC_SR04/hc_sr04.o ./Core/Src/HC_SR04/hc_sr04.su

.PHONY: clean-Core-2f-Src-2f-HC_SR04

