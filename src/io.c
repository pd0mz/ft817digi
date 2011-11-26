#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "io.h"
#include "port.h"

int
pinMode(uint8_t pin, uint8_t mode) {
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    volatile uint8_t *reg;

    if (port == NOT_A_PIN) return;

    reg = portModeRegister(port);

    if (mode == INPUT) {
        uint8_t oldSREG = SREG;
        cli();
        *reg &= ~bit;
        SREG = oldSREG;
    } else {
        uint8_t oldSREG = SREG;
        cli();
        *reg |= bit;
        SREG = oldSREG;
    }
}



int 
adcRead(uint8_t pin) {
    uint8_t lsb, msb;

    // Convert channel to pin number
    if (pin >= 14) pin -= 14;

#if defined(ADCSRB) && defined(MUX5)
    // The MUX5 bit of ADCSRB selects whether we're reading from channels
    // 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
    ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
#endif

    // Set the analog reference (high two bits of ADMUX) and select the
    // channel (low 4 bits). This also sets ADLAR (left-adjust result)
    // to 0 (the default).
#if defined(ADMUX)
    ADMUX = 0x40 | (pin & 0x07);
#endif

    // Start conversation
    sbi(ADCSRA, ADSC);

    // ADSC is cleared when the conversion finishes
    while (bit_is_set(ADCSRA, ADSC));

    // Read bits
    lsb = ADCL;
    msb = ADCH;

    return (msb << 8) | lsb;
}

void
adcWrite(uint8_t pin, int val) {
}
 
