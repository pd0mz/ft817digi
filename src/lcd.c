#include <stdlib.h>
#include "lcd.h"
#include "io.h"

lcd_t*
lcd_init(
    uint8_t db[],
    uint8_t rs,
    uint8_t rw,
    uint8_t enable,
    uint8_t rows,
    uint8_t cols) {
    
    lcd_t *LCD = (lcd_t *)malloc(sizeof(lcd_t));
    uint8_t i;

    LCD->db[0] = db[0];
    LCD->db[1] = db[1];
    LCD->db[2] = db[2];
    LCD->db[3] = db[3];
    LCD->rs = rs;
    LCD->rw = rw;
    LCD->enable = enable;
    LCD->rows = rows;
    LCD->cols = cols;    

    for (i = 0; i < 4; ++i) {
        pinMode(LCD->db[i], OUTPUT);
    }
    pinMode(LCD->rs, OUTPUT);
    pinMode(LCD->enable, OUTPUT);
    if (LCD->rw) {
        pinMode(LCD->rw, OUTPUT);
    }

    // Wait for power up
    _delay_ms(20);
    digitalWrite(LCD->rs, LOW);
    lcd_command_nibble(LCD, 0x03);
    _delay_ms(5);
    lcd_command_nibble(LCD, 0x03);
    _delay_us(200);
    lcd_command_nibble(LCD, 0x03);
    _delay_us(200);

    // Setup 4 bit mode
    lcd_command_nibble(LCD, 0x02);
    _delay_ms(5);
    // Set interface
    lcd_command_nibble(LCD, LCD_SET_4BIT_2LINE);
    // Don't shift display, hide cursor
    lcd_command_nibble(LCD, 0x08);
    // Clear display
    lcd_command_nibble(LCD, 0x01);
    // Entry mode
    lcd_command_nibble(LCD, 0x06);
    // Turn on display
    lcd_command_nibble(LCD, 0x0c);

    return LCD;
}

void
lcd_enable(lcd_t *LCD) {
    digitalWrite(LCD->enable, LOW);
    _delay_ms(1000);
    digitalWrite(LCD->enable, HIGH);
    _delay_ms(1000);
    digitalWrite(LCD->enable, LOW);
}

void
lcd_push_nibble(lcd_t *LCD, int value) {
    uint8_t nibble = value & 0x0f;
    uint8_t i;

    for (i = LCD->db[0]; i < LCD->db[3]; ++i) {
        digitalWrite(i, nibble & 0x01);
        nibble >>= 1;
    }
    lcd_enable(LCD);
}

void
lcd_push(lcd_t *LCD, uint8_t value) {
    uint8_t msb = value >> 4;
    uint8_t lsb = value & 0x0f;
    lcd_push_nibble(LCD, msb);
    lcd_push_nibble(LCD, lsb);
}

void
lcd_command_nibble(lcd_t *LCD, uint8_t value) {
    digitalWrite(LCD->rs, LOW);
    if (LCD->rw) {
        digitalWrite(LCD->rw, LOW);
    }
    lcd_push_nibble(LCD, value);
}

void
lcd_command(lcd_t *LCD, uint8_t value) {
    digitalWrite(LCD->rs, LOW);
    if (LCD->rw) {
        digitalWrite(LCD->rw, LOW);
    }
    lcd_push(LCD, value);
}

void
lcd_printch(lcd_t *LCD, uint8_t value) {
    digitalWrite(LCD->rs, HIGH);
    if (LCD->rw) {
        digitalWrite(LCD->rw, LOW);
    }
    lcd_push(LCD, value);
}

void
lcd_print(lcd_t *LCD, char msg[]) {
    uint8_t i;
    for (i = 0; i < strlen(msg); ++i) {
        lcd_printch(LCD, msg[i]);
    }
}

void
lcd_clear(lcd_t *LCD) {
    lcd_command(LCD, LCD_CMD_CLEAR);
    _delay_ms(1);
}

void
lcd_goto(lcd_t *LCD, uint8_t x, uint8_t y) {
    uint8_t i;
    
    // Shift x to match display rows
    for (i = 1; i < LCD->rows; ++i) {
        x += LCD->cols;
    }

    // Cursor go home
    lcd_command(LCD, LCD_CMD_HOME);
    // Advance right
    for (i = 0; i < x; ++i) {
        lcd_command(LCD, 0x14);
    }
}

void
lcd_scroll_left(lcd_t *LCD, uint8_t num) {
    uint8_t i;

    for (i = 0; i < num; ++i) {
        lcd_command(LCD, LCD_CMD_LEFT);
    }
}

