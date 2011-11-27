F_CPU       = 16000000UL
MODEL       = ATmega328
PIN_BUTTONS = 0

CFLAGS      = -O2 -DF_CPU=$(F_CPU) -D__AVR_$(MODEL)__ -DPIN_BUTTONS=$(PIN_BUTTONS)
