################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../source/flexspi_nor_flash_ops.c \
../source/flexspi_nor_polling_transfer.c \
../source/semihost_hardfault.c 

OBJS += \
./source/flexspi_nor_flash_ops.o \
./source/flexspi_nor_polling_transfer.o \
./source/semihost_hardfault.o 

C_DEPS += \
./source/flexspi_nor_flash_ops.d \
./source/flexspi_nor_polling_transfer.d \
./source/semihost_hardfault.d 


# Each subdirectory must supply rules for building sources it contributes
source/%.o: ../source/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -D__REDLIB__ -DCPU_MIMXRT1052DVL6B -DCPU_MIMXRT1052DVL6B_cm7 -DSERIAL_PORT_TYPE_UART=1 -DSDK_DEBUGCONSOLE=1 -DPRINTF_FLOAT_ENABLE=0 -DSCANF_FLOAT_ENABLE=0 -DPRINTF_ADVANCED_ENABLE=0 -DSCANF_ADVANCED_ENABLE=0 -DCR_INTEGER_PRINTF -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -I"/home/dave/Documents/MCUXpresso_11.1.0_3209/workspace/evkbimxrt1050_flexspi_nor_polling_transfer/board" -I"/home/dave/Documents/MCUXpresso_11.1.0_3209/workspace/evkbimxrt1050_flexspi_nor_polling_transfer/source" -I"/home/dave/Documents/MCUXpresso_11.1.0_3209/workspace/evkbimxrt1050_flexspi_nor_polling_transfer" -I"/home/dave/Documents/MCUXpresso_11.1.0_3209/workspace/evkbimxrt1050_flexspi_nor_polling_transfer/drivers" -I"/home/dave/Documents/MCUXpresso_11.1.0_3209/workspace/evkbimxrt1050_flexspi_nor_polling_transfer/device" -I"/home/dave/Documents/MCUXpresso_11.1.0_3209/workspace/evkbimxrt1050_flexspi_nor_polling_transfer/CMSIS" -I"/home/dave/Documents/MCUXpresso_11.1.0_3209/workspace/evkbimxrt1050_flexspi_nor_polling_transfer/utilities" -I"/home/dave/Documents/MCUXpresso_11.1.0_3209/workspace/evkbimxrt1050_flexspi_nor_polling_transfer/component/serial_manager" -I"/home/dave/Documents/MCUXpresso_11.1.0_3209/workspace/evkbimxrt1050_flexspi_nor_polling_transfer/component/lists" -I"/home/dave/Documents/MCUXpresso_11.1.0_3209/workspace/evkbimxrt1050_flexspi_nor_polling_transfer/component/uart" -I"/home/dave/Documents/MCUXpresso_11.1.0_3209/workspace/evkbimxrt1050_flexspi_nor_polling_transfer/xip" -O0 -fno-common -g3 -Wall -c  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin -fmerge-constants -fmacro-prefix-map="../$(@D)/"=. -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


