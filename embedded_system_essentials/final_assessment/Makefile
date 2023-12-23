#******************************************************************************
# Copyright (C) 2017 by Alex Fosdick - University of Colorado
#
# Redistribution, modification or use of this software in source or binary
# forms is permitted as long as the files maintain this copyright. Users are 
# permitted to modify this and use it to learn about the field of embedded
# software. Alex Fosdick and the University of Colorado are not liable for any
# misuse of this material. 
#
#*****************************************************************************

#------------------------------------------------------------------------------
# <Put a Description Here>
#
# Use: make [TARGET] [PLATFORM-OVERRIDES]
#
# Build Targets:
#      	<FILE>.O --- Build <FILE>.O Object file
#      	build --- Builds and links all source files
#      	all --- Same as build
#		clean --- Removes all generated files
#
# Platform Overrides:
#      	CPU - ARM Cortex Architecture (cortex-m0plus, cortex-m4)
#		ARCH - ARM Architecture (arm, thumb)
#		SPECS - Specs file to give the linker (nosys.specs, nano.specs)
#
#------------------------------------------------------------------------------
include sources.mk

# Platform Overrides
PLATFORM = MSP432

# Architectures Specific Flags
LINKER_FILE = msp432p401r.lds
CPU = cortex-m4
ARCH = thumb
SPECS = nosys.specs

# Compiler Flags and Defines
ifeq ($(PLATFORM),HOST)
	SRCS :=$(filter-out ./src/interrupts_msp432p401r_gcc.c, $(SRCS))
	SRCS :=$(filter-out ./src/system_msp432p401r.c, $(SRCS))
	SRCS :=$(filter-out ./src/startup_msp432p401r_gcc.c, $(SRCS))
	CC = gcc
	TARGET = c1m2
	LDFLAGS = -Wl,-Map=$(TARGET).map
	CFLAGS = -g -O0
	CPPFLAGs = -std=c99 -Wall -Werror -MD -MP
else
	CC = arm-none-eabi-gcc
	LD = arm-none-eabi-ld
	TARGET = c1m2
	LDFLAGS = -Wl,-Map=$(TARGET).map -T $(LINKER_FILE)
	CFLAGS = -g -O0 -mcpu=$(CPU) -m$(ARCH) --specs=$(SPECS) -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16
	CPPFLAGs = -std=c99 -Wall -Werror -MD -MP
endif

# generate dependencies
OBJS = $(SRCS:.c=.o)

%.o : %.c
	$(CC) -c $< $(INCLUDES) $(CFLAGS) $(CPPFLAGs) -o $@ -D$(PLATFORM) -DCOURSE1 -DVERBOSE

.PHONY: build
build: all

.PHONY: compile-all
compile-all: all

%.i : %.c 
	$(CC) -c $< $(INCLUDES) $(CFLAGS) $(CPPFLAGs) -E -o $@ -D$(PLATFORM)

%.asm : %.c
	$(CC) -S -c	$< $(INCLUDES) $(CFLAGS) $(CPPFLAGs) -o $@ -D$(PLATFORM)

.PHONY: all
all: $(TARGET).out

$(TARGET).out: $(OBJS)
	$(CC) $(OBJS) $(INCLUDES) $(CFLAGS) $(CPPFLAGs) $(LDFLAGS) -o $@ -D$(PLATFORM) -DCOURSE1 -DVERBOSE


.PHONY: clean
clean:
	rm -f $(OBJS) $(TARGET).out $(TARGET).map $(SRCS:.c=.asm) $(SRCS:.c=.i) $(SRCS:.c=.d)