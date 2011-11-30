#ifndef __MENU_H__
#define __MENU_H__

#ifndef PIN_BUTTONS
#error "Please define PIN_BUTTONS to indicate what ADC port the buttons use"
#endif

// Function prototypes
void menu_poll(unsigned char);

#endif
