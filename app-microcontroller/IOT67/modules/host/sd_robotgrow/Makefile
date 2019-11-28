#CROSS_COMPILE ?= arm-linux-gnueabihf-
CC = $(CROSS_COMPILE)gcc

CFLAGS = -Wall -O2 -Wextra

EXEC = dumper

all: $(EXEC)


OBJDIR = objs
SRC = $(shell find . -name \*.c)
OBJS = ${SRC:%.c=${OBJDIR}/%.o}
INCLUDES = includes/

${OBJDIR}/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "-- compiling" $<
	@$(CC) -MMD -MP -MF ${OBJDIR}/$*.d $(CFLAGS) $< -c -o $@ -I$(INCLUDES)

$(EXEC): $(OBJS)
	@echo "Linking $@ ..."
	@$(CC) $(LDFLAGS) -o $@ $(OBJS)
	@echo Done.


clean:
	find ${OBJDIR} -name "*.o" -exec rm {} \;
	find ${OBJDIR} -name "*.d" -exec rm {} \;

mrproper: clean
	rm -f $(EXEC)
