#ifndef __lcd_h__
#define __lcd_h__

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>

// Command bytes for LCD
#define LCD_CMD_CLEAR       0x01
#define LCD_CMD_HOME        0x02
#define LCD_CMD_BLANK       0x08
#define LCD_CMD_CURSOR_OFF  0x0c
#define LCD_CMD_CURSOR      0x0e
#define LCD_CMD_CURSOR_BIG  0x0f
#define LCD_CMD_RIGHT       0x1e
#define LCD_CMD_LEFT        0x18

#define LCD_SET_4BIT_1LINE  0x20
#define LCD_SET_4BIT_2LINE  0x28
#define LCD_SET_8BIT_1LINE  0x30
#define LCD_SET_8BIT_2LINE  0x38

// Pinout defaults
#ifndef PIN_LCD_D0
#define PIN_LCD_D0 4
#endif
#ifndef PIN_LCD_D1
#define PIN_LCD_D1 5
#endif
#ifndef PIN_LCD_D2
#define PIN_LCD_D2 6
#endif
#ifndef PIN_LCD_D3
#define PIN_LCD_D3 7
#endif
#ifndef PIN_LCD_RS
#define PIN_LCD_RS 8
#endif
#ifndef PIN_LCD_RW
#define PIN_LCD_RW 0
#endif
#ifndef PIN_LCD_EN
#define PIN_LCD_EN 9
#endif
#ifndef LCD_ROWS
#define LCD_ROWS   2
#endif
#ifndef LCD_COLS
#define LCD_COLS   16
#endif

typedef struct {
    uint8_t     db[4];
    uint8_t     rs;
    uint8_t     rw;
    uint8_t     enable;
    uint8_t     rows;
    uint8_t     cols;
} lcd_t;

void    lcd_enable(lcd_t *);
void    lcd_push_nibble(lcd_t *, int);
void    lcd_push(lcd_t *, uint8_t);
void    lcd_command_nibble(lcd_t *, uint8_t);
void    lcd_command(lcd_t *, uint8_t);
void    lcd_printch(lcd_t *, uint8_t);
void    lcd_print(lcd_t *, char[]);
void    lcd_clear(lcd_t *);

#endif // __lcd_h__
