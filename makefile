#/*
#    FreeRTOS V6.0.5 - Copyright (C) 2010 Real Time Engineers Ltd.
#
#    ***************************************************************************
#    *                                                                         *
#    * If you are:                                                             *
#    *                                                                         *
#    *    + New to FreeRTOS,                                                   *
#    *    + Wanting to learn FreeRTOS or multitasking in general quickly       *
#    *    + Looking for basic training,                                        *
#    *    + Wanting to improve your FreeRTOS skills and productivity           *
#    *                                                                         *
#    * then take a look at the FreeRTOS eBook                                  *
#    *                                                                         *
#    *        "Using the FreeRTOS Real Time Kernel - a Practical Guide"        *
#    *                  http://www.FreeRTOS.org/Documentation                  *
#    *                                                                         *
#    * A pdf reference manual is also available.  Both are usually delivered   *
#    * to your inbox within 20 minutes to two hours when purchased between 8am *
#    * and 8pm GMT (although please allow up to 24 hours in case of            *
#    * exceptional circumstances).  Thank you for your support!                *
#    *                                                                         *
#    ***************************************************************************
#
#    This file is part of the FreeRTOS distribution.
#
#    FreeRTOS is free software; you can redistribute it and/or modify it under
#    the terms of the GNU General Public License (version 2) as published by the
#    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
#    ***NOTE*** The exception to the GPL is included to allow you to distribute
#    a combined work that includes FreeRTOS without being obliged to provide the
#    source code for proprietary components outside of the FreeRTOS kernel.
#    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT
#    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
#    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
#    more details. You should have received a copy of the GNU General Public 
#    License and the FreeRTOS license exception along with FreeRTOS; if not it 
#    can be viewed here: http://www.freertos.org/a00114.html and also obtained 
#    by writing to Richard Barry, contact details for whom are available on the
#    FreeRTOS WEB site.
#
#    1 tab == 4 spaces!
#
#    http://www.FreeRTOS.org - Documentation, latest information, license and
#    contact details.
#
#    http://www.SafeRTOS.com - A version that is certified for use in safety
#    critical systems.
#
#    http://www.OpenRTOS.com - Commercial support, development, porting,
#    licensing and training services.
#*/

CC=arm-elf-gcc
OBJCOPY=arm-elf-objcopy
ARCH=arm-elf-ar
CRT0=boot.s
DEBUG=
OPTIM=-O2
LDSCRIPT=atmel-rom.ld

#
# CFLAGS common to both the THUMB and ARM mode builds
#

CFLAGS= \
-I.  \
-I./common/drivers/Atmel/at91lib/peripherals/usart \
-I./common/drivers/Atmel/at91lib/peripherals/pio \
-I./common/drivers/Atmel/at91lib/peripherals/aic \
-I./common/drivers/Atmel/at91lib/peripherals/spi \
-I./common/drivers/Atmel/at91lib/peripherals/pwmc \
-I./common/drivers/Atmel/at91lib/peripherals/adc \
-I./Drivers/USB \
-I./Drivers/I2C \
-I./Drivers/LCD \
-I./Drivers/PWM \
-I./Drivers/SPI \
-I./Drivers/UART \
-I./Drivers \
-I../Source/include  \
-I../Source/portable/GCC/ARM7_AT91SAM7S  \
-Wstrict-prototypes  \
-Wmissing-declarations  \
-Wno-strict-aliasing  \
-Winline \
-D SAM7_GCC  \
-D THUMB_INTERWORK \
-mthumb-interwork \
-mcpu=arm7tdmi  \
-T$(LDSCRIPT) \
$(DEBUG)  \
$(OPTIM) \
-fomit-frame-pointer 

THUMB_FLAGS=-mthumb
LINKER_FLAGS=-Xlinker -oe368rtos.elf -Xlinker -M -Xlinker -Map=e368rtos.map

#
# Source files that can be built to THUMB mode.
#
FREERTOS_THUMB_SRC= \
  ../Source/tasks.c \
  ../Source/queue.c \
  ../Source/list.c \
  ../Source/portable/GCC/ARM7_AT91SAM7S/port.c

E368_APP_THUMB_SRC= \
  ../Source/portable/MemMang/heap_2.c \
  main.c \
  ./Common/drivers/Atmel/at91lib/peripherals/usart/usart.c \
  ./Common/drivers/Atmel/at91lib/peripherals/pio/pio.c \
  ./Common/drivers/Atmel/at91lib/peripherals/aic/aic.c \
  ./Common/drivers/Atmel/at91lib/peripherals/spi/spi.c \
  ./Common/drivers/Atmel/at91lib/peripherals/pwmc/pwmc.c \
  ./Common/drivers/Atmel/at91lib/peripherals/adc/adc.c \
  Drivers/USB/USB-CDC.c \
  Drivers/UART/uartDriver.c \
  Drivers/LCD/lcdDriver.c \
  Drivers/SPI/spiDriver.c \
  Drivers/PWM/pwmDriver.c \
  Drivers/I2C/i2cDriver.c \
  e368.c \
  doublediv.c \
  terminal.c \
  sensors.c \
  setting.c \
  syscalls.c \
  Drivers/serial.c \
  lcdInterface.c \
  itoa.c \
  control.c \
  safety.c \
  Drivers/eeprom.c \


#
# Source files that must be built to ARM mode.
#
ARM_SRC= \
  ../Source/portable/GCC/ARM7_AT91SAM7S/portISR.c \
  Drivers/USB/USBIsr.c \
  Drivers/UART/uartIsr.c \
  Cstartup_SAM7.c  


#
# Define all object files.
#
ARM_OBJ = $(ARM_SRC:.c=.o)
FREERTOS_THUMB_OBJ = $(FREERTOS_THUMB_SRC:.c=.o)
DEMO_APP_THUMB_OBJ = $(E368_APP_THUMB_SRC:.c=.o)

e368rtos.bin : e368rtos.elf
	$(OBJCOPY) e368rtos.elf -O binary e368rtos.bin

e368rtos.hex : e368rtos.elf
	$(OBJCOPY) e368rtos.elf -O ihex e368rtos.hex

e368rtos.elf : $(ARM_OBJ) $(DEMO_APP_THUMB_OBJ) $(FREERTOS_THUMB_OBJ) $(CRT0) Makefile FreeRTOSConfig.h
	$(CC) $(CFLAGS) $(ARM_OBJ) $(DEMO_APP_THUMB_OBJ) $(FREERTOS_THUMB_OBJ) -nostartfiles $(CRT0) $(LINKER_FLAGS)

$(DEMO_APP_THUMB_OBJ)  : %.o : %.c $(LDSCRIPT) Makefile FreeRTOSConfig.h
	$(CC) -c $(THUMB_FLAGS) $(CFLAGS) $< -o $@


$(FREERTOS_THUMB_OBJ)  : %.o : %.c $(LDSCRIPT) Makefile FreeRTOSConfig.h
	$(CC) -c $(THUMB_FLAGS) $(CFLAGS) $< -o $@

$(ARM_OBJ) : %.o : %.c $(LDSCRIPT) Makefile FreeRTOSConfig.h
	$(CC) -c $(CFLAGS) $< -o $@

clean :
	rm -f $(ARM_OBJ)
	rm -f $(FREERTOS_THUMB_OBJ)
	rm -f $(DEMO_APP_THUMB_OBJ)
#	rm -f $(LWIP_THUMB_OBJ)
	rm -f e368rtos.bin e368rtos.elf

program :
	sam-ba \usb\ARM0 at91sam7x256-ek samba_flash.tcl > flash_log.log
	#cat flash_log.log
	#rm flash_log.log

program-jtag :
	sam-ba \\jlink\\ARM0 at91sam7x256-ek samba_flash.tcl > flash_log.log 2>&1
	cat flash_log.log
	rm -f flash_log.log

