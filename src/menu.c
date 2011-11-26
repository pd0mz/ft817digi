#define NUM_KEYS 5

#include "io.h"

enum {
    BUTTON_RIGHT,
    BUTTON_UP,
    BUTTON_DOWN,
    BUTTON_LEFT,
    BUTTON_SELECT
};

int _menu_adc_val[5] = {
    30, 150, 360, 535, 760
};

void
menu_init() {
    // Setup pin
    pinMode(PIN_BUTTONS, INPUT);
}

unsigned int
menu_getKey(void) {
    int adc = adcRead(PIN_BUTTONS);
    int i, key = -1;

    for (i = 0; i < NUM_KEYS; ++i) {
        if (adc < _menu_adc_val[i]) {
            key = i;
            break;
        }
    }

    if (i > NUM_KEYS) {
        key = -1;
    }
    
    return key;
} 
