F_CPU           = 16000000UL
MODEL           = ATmega328
PIN_BUTTONS     = 0

CFLAGS      = -O2 \
    -DF_CPU=$(F_CPU) \
    -mmcu=atmega328p \
    -Wl,-Tdata=0x801100,--defsym=__heap_end=0x80ffff \
    -D__AVR_$(MODEL)__ \
    -DPIN_BUTTONS=$(PIN_BUTTONS)

