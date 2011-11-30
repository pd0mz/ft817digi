include compiler.mk
include device.mk

INCLUDE     = -I$(shell pwd)/include
BUILD       = $(shell pwd)/build
PROG        = ft817plus
SRCS        = $(wildcard src/*.c)
OBJS        = $(SRCS:.c=.elf)
HEXS        = $(SRCS:.c=.hex)

.PHONY: all

all: dirs ${PROG}
	@echo "Compiled ${PROG}:"
	@ls -al ${PROG}.elf
	@ls -al ${PROG}.hex

dirs:
	mkdir -p $(BUILD)

${PROG}: ${PROG}.hex
	

${PROG}.hex: ${PROG}.elf
	$(OBJCOPY) -O $(BIN_FORMAT) -R .eeprom $(BUILD)/$< $(BUILD)/$@

${PROG}.elf: $(OBJS)
	$(CC) -o $(BUILD)/$@ $(CFLAGS) $(OBJS)

%.elf : %.c
	$(CC) -o $@ -c $< $(CFLAGS) $(INCLUDE)

clean:
	$(RM) $(OBJS) $(HEXS)
