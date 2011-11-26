#include <avr/io.h>
#include <util/delay.h>

#include "menu.h"
#include "tnc.h"

int main(void) {
    DDRB = 0xff;
    int counter;

    while (1) {
        PORTB = 0xFF;

        counter = 0;
        while (counter != 50) {
            _delay_loop_2(30000);
            counter++;
        }

        PORTB = 0x00;
        
        counter = 0;
        while (counter != 50) {
            _delay_loop_2(30000);
            counter++;
        }
    }

    return 1;
}

