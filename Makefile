# *****************************************************************************
# Author:   Kalvin
#
# File:	    Makefile
#
# The main Makefile for building the LTDZ firmware using GCC toolchain.
#
# Build the firmware:
#
#   $ make
#
# Flash the firmware using st-link v2:
#
#   $ make flash
#
# *****************************************************************************

FIRMWARE_VERSION = 119

######################################
# target
######################################

TARGET = ltdz_firmware

######################################
# building variables
######################################

# Debug build option: 0 | 1
DEBUG = 0

# Optimization level depending on DEBUG option
ifeq ($(DEBUG), 0)
OPT = -O3
else
OPT = -g -O0 -gdwarf-2
endif

#######################################
# paths
#######################################

# Build path
BUILD_DIR = build

# STM32 headers and source code
BSP_ROOT = ${HOME}/STM32Cube/Repository

# GCC compiler toolchain
GCC_PATH ?=

# GCC target compiler prefix
PREFIX = arm-none-eabi-

# ST Flash tool
ST_FLASH = st-flash

######################################
# source
######################################

# C includes
C_INCLUDES = \
-I./ \
-I$(BSP_ROOT)/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x \
-I$(BSP_ROOT)/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/STM32F10x_StdPeriph_Driver/inc \
-I$(BSP_ROOT)/STM32Cube_FW_F1_V1.8.3/Drivers/CMSIS/Include \
-I$(BSP_ROOT)/STM32Cube_FW_F1_V1.8.3/Drivers/CMSIS/Core/Include \
-I$(BSP_ROOT)/STM32Cube_FW_F1_V1.8.3/Drivers/CMSIS/Device/ST/STM32F1xx/Include \
-I$(BSP_ROOT)/STM32Cube_FW_F1_V1.8.3/Drivers/CMSIS/DSP/Include

# C sources
C_SOURCES = \
$(BSP_ROOT)/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_adc.c \
$(BSP_ROOT)/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_dma.c \
$(BSP_ROOT)/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_flash.c \
$(BSP_ROOT)/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c \
$(BSP_ROOT)/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c \
$(BSP_ROOT)/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_spi.c \
$(BSP_ROOT)/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_usart.c \
$(BSP_ROOT)/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/STM32F10x_StdPeriph_Driver/src/misc.c \
$(BSP_ROOT)/STM32Cube_FW_F1_V1.8.3/Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df1_init_q31.c \
$(BSP_ROOT)/STM32Cube_FW_F1_V1.8.3/Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df1_q31.c \
system_stm32f10x.c \
main.c

# AS includes
AS_INCLUDES =

# ASM sources
AS_SOURCES = \
startup_stm32f10x_md.s

#######################################
# Compiler toolchain definitions
#######################################

# The GCC compiler bin path can be either defined in make command via GCC_PATH
# variable (> make GCC_PATH=xxx) or it can be added to the PATH environment variable.

ifdef GCC_PATH
GCC_PREFIX = $(GCC_PATH)/$(PREFIX)
else
GCC_PREFIX = $(PREFIX)
endif

CC = $(GCC_PREFIX)gcc
AS = $(GCC_PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PREFIX)objcopy
SZ = $(GCC_PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#######################################
# CFLAGS
#######################################

# cpu
CPU = -mcpu=cortex-m3

# fpu
# NONE for Cortex-M0/M0+/M3

# float-abi

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# C defines
C_DEFS = \
-DUSE_STDPERIPH_DRIVER \
-DSTM32F103xB \
-DSTM32F10X_MD \
-DARM_MATH_CM3 \
-DFIRMWARE_VERSION=$(FIRMWARE_VERSION)

# C compiler options
CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT)
CFLAGS += -Wall -Werror -Wextra -Wpedantic
CFLAGS += -fdata-sections
CFLAGS += -ffreestanding
CFLAGS += -ffunction-sections
CFLAGS += -fno-common
CFLAGS += -fno-builtin
CFLAGS += -nostdlib

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

# AS defines
AS_DEFS =

# AS flags
ASFLAGS += $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT)
ASFLAGS += -Wall
ASFLAGS += -fdata-sections
ASFLAGS += -ffunction-sections

#######################################
# LDFLAGS
#######################################

# Linker script
LDSCRIPT = STM32F103C8Tx_FLASH.ld

# Libraries
LIBS = -lc -lm -lnosys

# Extra libraries
LIBDIR =

# Linker options
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

#######################################
# build the application
#######################################

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(AS_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(AS_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir $@

#######################################
# Clean up
#######################################

clean:
	-rm -fR $(BUILD_DIR)

#######################################
# Flash the binary using st-link v2
#######################################

flash:
	$(ST_FLASH) --reset write $(BUILD_DIR)/$(TARGET).bin 0x08000000

#######################################
# Dependencies
#######################################

-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
