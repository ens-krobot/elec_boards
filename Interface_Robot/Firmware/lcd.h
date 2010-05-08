/**
 * @file lcd.h
 * Bibliothèque de gestion de l'afficheur LCD CLDC204
*/

#ifndef LCD_H
#define LCD_H

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"
#include <i2c.h>

#define ADD_LCD 0x07

// Commandes de l'afficheur LCD
#define   CMD_HEAD            0x1B
#define   CMD_CLEAR           0x43
#define   CMD_CURSOR_ON       0x53
#define   CMD_CURSOR_OFF      0x73
#define   CMD_BACKLIGHT_ON    0x42
#define   CMD_BACKLIGHT_OFF   0x62
#define   CMD_GOTO_ORIGIN     0x48
#define   CMD_GOTO_POS        0x4C

void lcd_init(BYTE address);
void lcd_clear(BYTE address);
void lcd_set_cursor(BYTE address, BOOL activate_cursor);
void lcd_set_backlight(BYTE address, BOOL activate_backlight);
void lcd_goto_origin(BYTE address);
void lcd_goto(BYTE address, BYTE xx, BYTE yy);
void lcd_write(BYTE address, unsigned char* msg);
void lcd_write_line(BYTE address, BYTE line, unsigned char *msg);


#endif
