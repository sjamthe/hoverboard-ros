######################################
# target
######################################
TARGET = hover

######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og

# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_flash.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_rcc.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_tim.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_cortex.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_gpio.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_pwr.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_adc.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_uart.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_i2c.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_dma.c \
Src/system_at32f4xx.c \
Src/setup.c \
Src/main.c \
Src/bldc.c \
Src/stm32f1xx_it.c \
Src/BLDC_controller_data.c \
Src/BLDC_controller.c \
Src/hallinterrupts.c \
Src/motorcontrol.c

# CPP sources
CPP_SOURCES = \
./ROSSerial/duration.cpp \
./ROSSerial/time.cpp \
./Src/ROS_main.cpp \
./Src/ROS_subscribe.cpp 

# ASM sources
ASM_SOURCES =  \
startup_at32f403xe.s

#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
CXX = $(PREFIX)g++
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
AR = $(PREFIX)ar
SZ = $(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
# NONE for Cortex-M0/M0+/M3
FPU=-mfpu=fpv4-sp-d16
# float-abi
FLOAT-ABI=-mfloat-abi=softfp

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS =

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DAT32F403Rx_HD


# AS includes
AS_INCLUDES =

# C includes
C_INCLUDES =  \
-IInc \
-IDrivers/Modified_HAL_Driver/Inc \
-IDrivers/CMSIS/CM4/DeviceSupport \
-IDrivers/CMSIS/CM4/CoreSupport \
-IROSSerial \
-IROSSerial/ros \
-IROSSerial/rosserial_msgs \
-IROSSerial/std_msgs 

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -std=gnu11

CXXFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(CXX_INCLUDES) $(OPT) -Wall  -fno-rtti
CXXFLAGS += -fdata-sections -ffunction-sections -fpermissive

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
CXXFLAGS += -g2 -gdwarf-2 
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"
CXXFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = AT32F403RCTx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys
LIBDIR =
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -specs=nosys.specs

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of CPP objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR) 
	$(CXX) -c $(CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.c Inc/config.h Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@ -include Inc/stm32f1xx_conf.h

$(BUILD_DIR)/%.o: %.s Inc/config.h Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir -p $@

format:
	find Src/ Inc/ -iname '*.h' -o -iname '*.c' | xargs clang-format -i
#######################################
# clean up
#######################################
clean:
	-rm -fR .dep $(BUILD_DIR)

load:
	openocd -f interface/stlink-v2.cfg -f target/stm32f3x.cfg -c init -c "reset halt" -c "stm32f1x unlock 0" \
	 -c "flash write_image erase $(BUILD_DIR)/$(TARGET).hex 0 ihex" -c "shutdown"

connect:
	openocd -f interface/stlink-v2.cfg -f target/stm32f3x.cfg -c init 

run:
	openocd -f interface/stlink-v2.cfg -f target/stm32f3x.cfg -c init -s "reset init" -c "reset run" -c "shutdown"

unlock:
	openocd -f interface/stlink.cfg -f target/stm32f3x.cfg -c init -c "reset halt" -c "stm32f1x unlock 0"

#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***
