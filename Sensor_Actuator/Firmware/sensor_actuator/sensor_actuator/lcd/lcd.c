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

#include "lcd.h"

static i2c_lcd_ctx _lcd_ctx;
i2c_lcd_ctx *lcd_ctx = &_lcd_ctx;
void lcd_write_command(uint8_t *data, uint8_t size);


i2c_lcd_ctx *lcd_init(I2c *i2c, uint8_t address) {
  lcd_ctx->i2c = i2c;
  lcd_ctx->address = address;

  return lcd_ctx;
}

void lcd_write_command(uint8_t *data, uint8_t size) {
  i2c_start_w(lcd_ctx->i2c, lcd_ctx->address, size, I2C_STOP);
  i2c_write(lcd_ctx->i2c, data, size);
}

void lcd_cls(void) {
  uint8_t command[] = {CMD_HEAD, CMD_CLEAR};

  lcd_write_command(command, sizeof(command)/sizeof(uint8_t));
}

void lcd_set_cursor(uint8_t action) {
  uint8_t command[] = {CMD_HEAD, CMD_CURSOR_ON};

  if (action == DEACTIVE)
    command[1] = CMD_CURSOR_OFF;

  lcd_write_command(command, sizeof(command)/sizeof(uint8_t));
}

void lcd_set_backlight(uint8_t action) {
  uint8_t command[] = {CMD_HEAD, CMD_BACKLIGHT_ON};

  if (action == DEACTIVE)
    command[1] = CMD_BACKLIGHT_OFF;

  lcd_write_command(command, sizeof(command)/sizeof(uint8_t));
}

void lcd_goto_origin(void) {
    uint8_t command[] = {CMD_HEAD, CMD_GOTO_ORIGIN};

    lcd_write_command(command, sizeof(command)/sizeof(uint8_t));
}

void lcd_goto(uint8_t x, uint8_t y) {
  uint8_t command[4] = {CMD_HEAD, CMD_GOTO_POS, 0, 0};
  command[2] = x;
  command[3] = y;

  lcd_write_command(command, sizeof(command)/sizeof(uint8_t));
}

void lcd_write(unsigned char *msg) {
  lcd_write_command(msg, strlen((char*)msg));
}

void lcd_write_line(unsigned char *msg, uint8_t line) {
  uint8_t command[1];
  command[0] = line;

  lcd_write_command(command, sizeof(command)/sizeof(uint8_t));
  lcd_write_command(msg, strlen((char*)msg));
}

void lcd_set_data(uint8_t id, unsigned char data[]) {
  uint8_t line, start;
  line = id / 3;
  start = 7*(id % 3);
  for (unsigned int i=0; i < 7; i++) {
    lcd_ctx->data_lines[line][start+i] = data[i];
  }
}

void lcd_refresh_line(unsigned char line) {
  if (line >= 1 && line <= 4) {
    lcd_write_line(lcd_ctx->data_lines[line-1], line);
  }
}
