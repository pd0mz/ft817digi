CC          = avr-gcc
AVRDUDE     = avrdude -F -V
OBJCOPY     = avr-objcopy
BIN_FORMAT  = ihex

F_CPU       = 16000000UL
MODEL       = ATmega168
PIN_BUTTONS = 0

CFLAGS      = -O2 -DF_CPU=$(F_CPU) -D__AVR_$(MODEL)__ -DPIN_BUTTONS=$(PIN_BUTTONS)
INCLUDE     = -I$(shell pwd)/../include
PROG        = ft817plus
SRCS        = $(wildcard *.c)
ELFS        = $(subst .c,.elf,$(SRCS))

.PHONY: all

all:
	$(MAKE) ${PROG}

${PROG}: ${PROG}.hex
	

${PROG}.hex: ${PROG}.elf
	$(OBJCOPY) -O $(BIN_FORMAT) -R .eeprom $< $@

${PROG}.elf: $(ELFS)
	$(CC) -o $@ $(ELFS)

%.elf : %.c
	$(CC) -o $@ $< -c $(CFLAGS) $(INCLUDE)

clean:
	$(RM) *.elf *.hex ${PROG}
