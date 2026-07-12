################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Custom_CLI.c \
../Core/Src/Custom_utils.c \
../Core/Src/HW_Encoders.c \
../Core/Src/HW_EndEffector.c \
../Core/Src/HW_LimitSwitches.c \
../Core/Src/HW_Motors.c \
../Core/Src/PoE_Kinematics.c \
../Core/Src/PoE_helper.c \
../Core/Src/PoE_lieTheory.c \
../Core/Src/PoE_matrixManip.c \
../Core/Src/Robot_Control.c \
../Core/Src/main.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/Custom_CLI.o \
./Core/Src/Custom_utils.o \
./Core/Src/HW_Encoders.o \
./Core/Src/HW_EndEffector.o \
./Core/Src/HW_LimitSwitches.o \
./Core/Src/HW_Motors.o \
./Core/Src/PoE_Kinematics.o \
./Core/Src/PoE_helper.o \
./Core/Src/PoE_lieTheory.o \
./Core/Src/PoE_matrixManip.o \
./Core/Src/Robot_Control.o \
./Core/Src/main.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/Custom_CLI.d \
./Core/Src/Custom_utils.d \
./Core/Src/HW_Encoders.d \
./Core/Src/HW_EndEffector.d \
./Core/Src/HW_LimitSwitches.d \
./Core/Src/HW_Motors.d \
./Core/Src/PoE_Kinematics.d \
./Core/Src/PoE_helper.d \
./Core/Src/PoE_lieTheory.d \
./Core/Src/PoE_matrixManip.d \
./Core/Src/Robot_Control.d \
./Core/Src/main.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/CMSIS_DSP/Include -I../Drivers/CMSIS_DSP/PrivateInclude -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Custom_CLI.cyclo ./Core/Src/Custom_CLI.d ./Core/Src/Custom_CLI.o ./Core/Src/Custom_CLI.su ./Core/Src/Custom_utils.cyclo ./Core/Src/Custom_utils.d ./Core/Src/Custom_utils.o ./Core/Src/Custom_utils.su ./Core/Src/HW_Encoders.cyclo ./Core/Src/HW_Encoders.d ./Core/Src/HW_Encoders.o ./Core/Src/HW_Encoders.su ./Core/Src/HW_EndEffector.cyclo ./Core/Src/HW_EndEffector.d ./Core/Src/HW_EndEffector.o ./Core/Src/HW_EndEffector.su ./Core/Src/HW_LimitSwitches.cyclo ./Core/Src/HW_LimitSwitches.d ./Core/Src/HW_LimitSwitches.o ./Core/Src/HW_LimitSwitches.su ./Core/Src/HW_Motors.cyclo ./Core/Src/HW_Motors.d ./Core/Src/HW_Motors.o ./Core/Src/HW_Motors.su ./Core/Src/PoE_Kinematics.cyclo ./Core/Src/PoE_Kinematics.d ./Core/Src/PoE_Kinematics.o ./Core/Src/PoE_Kinematics.su ./Core/Src/PoE_helper.cyclo ./Core/Src/PoE_helper.d ./Core/Src/PoE_helper.o ./Core/Src/PoE_helper.su ./Core/Src/PoE_lieTheory.cyclo ./Core/Src/PoE_lieTheory.d ./Core/Src/PoE_lieTheory.o ./Core/Src/PoE_lieTheory.su ./Core/Src/PoE_matrixManip.cyclo ./Core/Src/PoE_matrixManip.d ./Core/Src/PoE_matrixManip.o ./Core/Src/PoE_matrixManip.su ./Core/Src/Robot_Control.cyclo ./Core/Src/Robot_Control.d ./Core/Src/Robot_Control.o ./Core/Src/Robot_Control.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

