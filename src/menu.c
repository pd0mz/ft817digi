#define NUM_KEYS 5

#include "io.h"

enum {
    BUTTON_RIGHT,
    BUTTON_UP,
    BUTTON_DOWN,
    BUTTON_LEFT,
    BUTTON_SELECT
};

unsigned int menu_adc_val[5] = {
    30, 150, 360, 535, 760
};

unsigned char menu_active;          // Is the menu visible?


void
menu_init() {
    // Setup pin
    pinMode(PIN_BUTTONS, INPUT);
}

unsigned int
menu_get_key(void) {
    int adc = adcRead(PIN_BUTTONS);
    int i, key = -1;

    for (i = 0; i < NUM_KEYS; ++i) {
        if (adc < menu_adc_val[i]) {
            key = i;
            break;
        }
    }

    if (i > NUM_KEYS) {
        key = -1;
    }
    
    return key;
} 

void
menu_poll(unsigned char state) {
    int key = menu_get_key();
    if (menu_active == 0) {
        if (key == BUTTON_SELECT) {
            menu_active = 1;
        }
    }
}
    
