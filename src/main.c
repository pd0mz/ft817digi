#include <avr/io.h>
#include <util/delay.h>

#include "menu.h"
#include "lcd.h"
#include "tnc.h"

// State
//  - b00000000
//  - b01000000 TNC enabled
//  - b10000000 Serial passthrough enabled
unsigned char state;

// LCD
lcd_t *display;

void setup(void) {
    uint8_t db[4] = {PIN_LCD_D0, PIN_LCD_D1, PIN_LCD_D2, PIN_LCD_D3};
    display = lcd_init(
        db,
        PIN_LCD_RS,
        PIN_LCD_RW,
        PIN_LCD_EN,
        LCD_ROWS,
        LCD_COLS);
    menu_init();
    tnc_init();

    state = 0;
}

void loop(void) {
    switch (state) {
        case 0x40:
            break;
        case 0x80:
            tnc_loop();
            return;
            break;
    }

    // Still here?
    menu_poll(state);
}

int main(void) {
    setup();
    while (1) {
        loop();
    }
    return 0;
}
