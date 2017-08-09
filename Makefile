TARGET=main
EXECUTABLE=main.elf
BINARYOUT=main.bin
STLINK=~/src/stm32/stlink
FLASH_SCRIPT=./flash_remote.sh
LIBPATH=/home/ali/src/stm32/STM32F4xx_DSP_StdPeriph_Lib_V1.8.0

CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump

BIN=$(CP) -O ihex 

#DEFS = -DUSE_STDPERIPH_DRIVER -DSTM32F4XX -DHSE_VALUE=8000000
DEFS = -DUSE_STDPERIPH_DRIVER -DSTM32F40_41xxx -DSTM32F407VG -DHSE_VALUE=8000000
STARTUP = $(LIBPATH)/Libraries/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc_ride7/startup_stm32f40xx.s

MCU = cortex-m4
MCFLAGS = -mcpu=$(MCU) -mthumb -mlittle-endian -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb-interwork -std=c99 -g
STM32_INCLUDES = -I$(LIBPATH)/Libraries/CMSIS/Device/ST/STM32F4xx/Include/ \
	-I$(LIBPATH)/Libraries/CMSIS/Include/ \
	-I$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/inc/ \
	-I./drivers/STM32F4xx_HAL_Driver/Inc/ \
	-I./drivers/BME280_driver/ \
	-I./drivers/tm/ \
	-I./inc/


OPTIMIZE       = -Og

CFLAGS	= $(MCFLAGS)  $(OPTIMIZE)  $(DEFS) -I. -I./ $(STM32_INCLUDES)  -Wl,-T,stm32_flash.ld
AFLAGS	= $(MCFLAGS) 
#-mapcs-float use float regs. small increase in code size

SRC = ./src/main.c \
	./src/flash.c \
	./src/i2c.c \
	./src/rtc.c \
	./src/gpio.c \
	./src/adc.c \
	./src/onewire.c \
	./src/command_parser.c \
	./src/system_stm32f4xx.c \
	./drivers/BME280_driver/bme280.c \
	./drivers/BME280_driver/bme280_support.c \
	./drivers/tm/tm_stm32f4_delay.c \
	./drivers/tm/tm_stm32f4_usart.c \
	./drivers/tm/tm_stm32f4_ds18b20.c \
	./drivers/tm/tm_stm32f4_disco.c \
	./drivers/tm/tm_stm32f4_watchdog.c \
	./drivers/tm/tm_stm32f4_adc.c \
	./drivers/tm/tm_stm32f4_rtc.c \
	./drivers/tm/tm_stm32f4_gpio.c \
	./drivers/tm/tm_stm32f4_onewire.c \
	./drivers/tm/tm_stm32f4_timer_properties.c \
	./drivers/tm/tm_stm32f4_pwm.c \
	./drivers/tm/tm_stm32f4_pwmin.c \
	./drivers/tm/tm_stm32f4_exti.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/misc.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_adc.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_can.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_crc.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_aes.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_des.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_tdes.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dac.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dbgmcu.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dcmi.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_exti.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_flash.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_fsmc.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash_md5.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash_sha1.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_iwdg.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_pwr.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rng.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rtc.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_sdio.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_syscfg.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_wwdg.c 

OBJDIR = .
OBJ = $(SRC:%.c=$(OBJDIR)/%.o) 
OBJ += Startup.o

all: $(TARGET)

$(TARGET): $(EXECUTABLE)
	$(CP) -O ihex $^ $@
	$(CP) -O binary $(EXECUTABLE) $(BINARYOUT)

$(EXECUTABLE): $(SRC) $(STARTUP)
	$(CC) $(CFLAGS) $^ -lm -lc -lnosys -o $@

flash:
	$(FLASH_SCRIPT) $(BINARYOUT) 0x8000000

burn: 
	$(STLINK)/st-flash write $(BINARYOUT) 0x8000000

reset: 
	echo 'reset halt' | nc growpi 4444

clean:
	rm -f Startup.lst  $(TARGET)  $(TARGET).lst $(OBJ) $(AUTOGEN)  $(TARGET).out  $(TARGET).hex  $(TARGET).map \
	 $(TARGET).dmp  $(TARGET).elf  $(TARGET).bin
