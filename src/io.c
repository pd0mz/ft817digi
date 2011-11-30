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

static void turnOffPWM(uint8_t timer)
{
    switch (timer)
    {
        #if defined(TCCR1A) && defined(COM1A1)
        case TIMER1A:   cbi(TCCR1A, COM1A1);    break;
        #endif
        #if defined(TCCR1A) && defined(COM1B1)
        case TIMER1B:   cbi(TCCR1A, COM1B1);    break;
        #endif

        #if defined(TCCR2) && defined(COM21)
        case  TIMER2:   cbi(TCCR2, COM21);      break;
        #endif

        #if defined(TCCR0A) && defined(COM0A1)
        case  TIMER0A:  cbi(TCCR0A, COM0A1);    break;
        #endif

        #if defined(TIMER0B) && defined(COM0B1)
        case  TIMER0B:  cbi(TCCR0A, COM0B1);    break;
        #endif
        #if defined(TCCR2A) && defined(COM2A1)
        case  TIMER2A:  cbi(TCCR2A, COM2A1);    break;
        #endif
        #if defined(TCCR2A) && defined(COM2B1)
        case  TIMER2B:  cbi(TCCR2A, COM2B1);    break;
        #endif

        #if defined(TCCR3A) && defined(COM3A1)
        case  TIMER3A:  cbi(TCCR3A, COM3A1);    break;
        #endif
        #if defined(TCCR3A) && defined(COM3B1)
        case  TIMER3B:  cbi(TCCR3A, COM3B1);    break;
        #endif
        #if defined(TCCR3A) && defined(COM3C1)
        case  TIMER3C:  cbi(TCCR3A, COM3C1);    break;
        #endif

        #if defined(TCCR4A) && defined(COM4A1)
        case  TIMER4A:  cbi(TCCR4A, COM4A1);    break;
        #endif
        #if defined(TCCR4A) && defined(COM4B1)
        case  TIMER4B:  cbi(TCCR4A, COM4B1);    break;
        #endif
        #if defined(TCCR4A) && defined(COM4C1)
        case  TIMER4C:  cbi(TCCR4A, COM4C1);    break;
        #endif
        #if defined(TCCR5A)
        case  TIMER5A:  cbi(TCCR5A, COM5A1);    break;
        case  TIMER5B:  cbi(TCCR5A, COM5B1);    break;
        case  TIMER5C:  cbi(TCCR5A, COM5C1);    break;
        #endif
    }
}


void
digitalWrite(uint8_t pin, uint8_t val) {
    uint8_t timer = digitalPinToTimer(pin);
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    volatile uint8_t *out;

    if (port == NOT_A_PIN) return;

    // If the pin that support PWM output, we need to turn it off
    // before doing a digital write.
    if (timer != NOT_ON_TIMER) turnOffPWM(timer);

    out = portOutputRegister(port);

    if (val == LOW) {
        uint8_t oldSREG = SREG;
                cli();
        *out &= ~bit;
        SREG = oldSREG;
    } else {
        uint8_t oldSREG = SREG;
                cli();
        *out |= bit;
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
 
