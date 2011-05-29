/**
 * USB-CAN converter
 *
 * This file contains the interface for a very simple serial driver for STM32
 *
 * Copyright © 2011 Nicolas Dandrimont <olasd@crans.org>
 * Authors: Nicolas Dandrimont <olasd@crans.org>
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

#ifndef SERIAL_H
#define SERIAL_H

#include <inttypes.h>
#include <string.h>

#include <cpu/irq.h>
#include <cpu/power.h>

#include <drv/clock_stm32.h>
#include <drv/gpio_stm32.h>
#include <drv/irq_cm3.h>
#include <drv/timer.h>

#include "hw/hw_led.h"

#include <io/stm32_uart.h>

#include <kern/sem.h>

struct stm32_usart *serial_init(unsigned long baudrate);
void serial_deinit(void);

int serial_getchar(void);

void serial_putchar(char ch);
void serial_transmit(const char *buffer, size_t len);


#endif
