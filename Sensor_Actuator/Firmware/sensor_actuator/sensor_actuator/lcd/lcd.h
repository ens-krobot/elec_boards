/**
 * I2C LCD wrapper for the USB CAN Board
 *
 * Copyright © 2012 Xavier Lagorce <Xavier.Lagorce@crans.org>
 * Authors: Xavier Lagorce <Xavier.Lagorce@crans.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef LCD_H
#define LCD_H

#include <drv/i2c.h>
#include <drv/timer.h>
#include "string.h"

#define ACTIVE   1
#define DEACTIVE 2

#define   CMD_HEAD            0x1B
#define   CMD_CLEAR           0x43
#define   CMD_CURSOR_ON       0x53
#define   CMD_CURSOR_OFF      0x73
#define   CMD_BACKLIGHT_ON    0x42
#define   CMD_BACKLIGHT_OFF   0x62
#define   CMD_GOTO_ORIGIN     0x48
#define   CMD_GOTO_POS        0x4C

typedef struct {
  I2c *i2c;
  uint8_t address;
  uint8_t data_lines[4][21];
} i2c_lcd_ctx;

i2c_lcd_ctx *lcd_init(I2c *i2c, uint8_t address);
void lcd_cls(void);
void lcd_set_cursor(uint8_t action);
void lcd_set_backlight(uint8_t action);
void lcd_goto_origin(void);
void lcd_goto(uint8_t x, uint8_t y);
void lcd_write(unsigned char *msg);
void lcd_write_line(unsigned char *msg, uint8_t line);

void lcd_set_data(uint8_t id, unsigned char data[]);
void lcd_refresh_line(unsigned char line);

#endif
