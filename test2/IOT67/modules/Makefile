# Makefile for LPC122x micro-controller based boards

TARGET_DIR = apps/$(MODULE)/$(NAME)

# If you use another of the LPC122x micro-controllers, copy the lpc_link_lpc1224.ld file
# to match you LPC micro-controller model and modify the memory sizes definitions at the
# beginning of your new file.
# Note that the LPC1224 has the smallest memory of the LPC122x branch, and thus using the
# lpc1224 linker script will work for all LPC122x, but not all memory will be available.
LPC = lpc1224
CPU = cortex-m0
ARCH = armv6-m

CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
# DEBUG = -ggdb3
LD_DEBUG = $(DEBUG)
#LD_DEBUG = $(DEBUG) -Wl,--print-memory-usage
#LD_DEBUG = $(DEBUG) -Wl,--print-gc-sections -Wl,--print-output-format \
		   -Wl,--print-memory-usage
FOPTS = -fno-builtin -ffunction-sections -fdata-sections -ffreestanding
CFLAGS = -Wall -O2 $(DEBUG) -mthumb -mcpu=$(CPU) $(FOPTS)
LDFLAGS = -static $(LD_DEBUG) -nostartfiles -nostdlib -Tlpc_link_$(LPC).ld \
		  -Wl,--gc-sections -Wl,--sort-section=alignment -Wl,--build-id=none \
		  -Wl,-Map=$(TARGET_DIR)/lpc_map_$(LPC).map


APPS = $(subst apps/,,$(wildcard apps/*/*))

.PHONY: all $(APPS)
all: $(APPS)

INCLUDES = include/
TARGET_INCLUDES = $(TARGET_DIR)/
OBJDIR = objs

C_SRC = $(wildcard */*.c)
C_SRC += $(wildcard lib/*/*.c)
C_SRC += $(wildcard lib/protocols/*/*.c)

A_SRC = $(wildcard */*.s)
A_SRC += $(wildcard lib/*/*.s)
A_SRC += $(wildcard lib/protocols/*/*.s)


OBJS = ${C_SRC:%.c=${OBJDIR}/%.o} ${A_SRC:%.s=${OBJDIR}/%.o}
DEPS = ${OBJS:%.o=$(OBJDIR)/%.d}

NAME_SRC = $(wildcard $(TARGET_DIR)/*.c)
NAME_A_SRC = $(wildcard $(TARGET_DIR)/*.s)
NAME_OBJS = ${NAME_SRC:%.c=${OBJDIR}/%.o} ${NAME_A_SRC:%.s=${OBJDIR}/%.o}
NAME_DEPS = ${NAME_OBJS:%.o=$(OBJDIR)/%.d}

-include $(DEPS) $(NAME_DEPS)

.SECONDARY: $(OBJS) $(NAME_OBJS)
.PRECIOUS: %.elf
%.elf: $(OBJS) $(NAME_OBJS)
	@echo "Linking $(MODULE)/$(NAME) ..."
	@$(CC) $(LDFLAGS) $(OBJS) $(NAME_OBJS) -o $@

%.bin: %.elf
	@echo "Creating image : [32m$@[39m"
	@$(CROSS_COMPILE)objcopy -R .stack -R .bss -O binary $^ $@
	@ls -l $@
	@$(CROSS_COMPILE)size $^
	@echo Done.

${OBJDIR}/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "-- compiling" $<
	@$(CC) -MMD -MP -MF ${OBJDIR}/$*.d $(CFLAGS) $< -c -o $@ -I$(INCLUDES) -I$(TARGET_INCLUDES)

${OBJDIR}/%.o: %.s
	@mkdir -p $(dir $@)
	@echo "-- assembing" $<
	@$(CC) -MMD -MP -MF ${OBJDIR}/$*.d $(CFLAGS) $< -c -o $@ -I$(INCLUDES) -I$(TARGET_INCLUDES)


$(APPS):
	@make --no-print-directory MODULE=$(shell dirname $@) NAME=$(shell basename $@) apps/$(shell dirname $@)/$(shell basename $@)/$(shell basename $@).bin

all_apps: $(APPS)

clean:
	rm -rf $(OBJDIR)

mrproper: clean
	rm -f apps/*/*/*.bin apps/*/*/*.elf apps/*/*/*.map


# Some notes :
# The command "make -f /path/to/here/Makefile module/app_name" does not work, as well
# as the command "make -f /path/to/here/apps/module/app_name/Makefile".
# This could be solved in the app Makefiles by replacing
#   "NAME = $(shell basename $(CURDIR))"
# with
#   "NAME = $(shell basename $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST)))))"
# and possibly some similar trick for the base Makefile but this is just
# unreadable and moslty unusefull, so it won't be supported.
# Use "make -C /path/to/here/ module/app_name" or "make -C /path/to/here/apps/module/app_name"
# instead.

