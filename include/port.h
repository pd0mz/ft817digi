#ifndef __PORT_H__
#define __PORT_H__

#include <avr/pgmspace.h>
#include <avr/io.h>

#define NOT_A_PIN   0
#define NOT_A_PORT  0

#define NOT_ON_TIMER 0
#define TIMER0A     0x01
#define TIMER0B     0x02
#define TIMER1A     0x03
#define TIMER1B     0x04
#define TIMER2      0x05
#define TIMER2A     0x06
#define TIMER2B     0x07

#define TIMER3A     0x08
#define TIMER3B     0x09
#define TIMER3C     0x0a
#define TIMER4A     0x0b
#define TIMER4B     0x0c
#define TIMER4C     0x0d
#define TIMER5A     0x0e
#define TIMER5B     0x0f
#define TIMER5C     0x10

const static uint8_t SS   = 10;
const static uint8_t MOSI = 11;
const static uint8_t MISO = 12;
const static uint8_t SCK  = 13;

extern const uint16_t PROGMEM port_to_mode_PGM[];
extern const uint16_t PROGMEM port_to_input_PGM[];
extern const uint16_t PROGMEM port_to_output_PGM[];

extern const uint8_t PROGMEM digital_pin_to_port_PGM[];
extern const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[];
extern const uint8_t PROGMEM digital_pin_to_timer_PGM[];

// Get the bit location within the hardware port of the given virtual pin.
// This comes from the pins_*.c file for the active board configuration.
// 
// These perform slightly better as macros compared to inline functions
//
#define digitalPinToPort(P) ( pgm_read_byte( digital_pin_to_port_PGM + (P) ) )
#define digitalPinToBitMask(P) ( pgm_read_byte( digital_pin_to_bit_mask_PGM + (P) ) )
#define digitalPinToTimer(P) ( pgm_read_byte( digital_pin_to_timer_PGM + (P) ) )
#define analogInPinToBit(P) (P)
#define portOutputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_output_PGM + (P))) )
#define portInputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_input_PGM + (P))) )
#define portModeRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_mode_PGM + (P))) )

#ifdef __cplusplus
extern "C" {
#endif

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#ifdef __cplusplus
} // extern "C"
#endif

#endif // __PORT_H__
