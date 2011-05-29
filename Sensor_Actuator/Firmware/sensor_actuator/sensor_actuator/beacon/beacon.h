/**
 * Rotary beacon
 *
 * Header file for rotary beacon logic.
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
#ifndef ROTARY_BEACON_H
#define ROTARY_BEACON_H

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include <hw/hw_led.h>
#include <drv/can.h>
#include <drv/irq_cm3.h>
#include <drv/timer.h>
#include <drv/clock_stm32.h>
#include <drv/gpio_stm32.h>
#include <mware/event.h>
#include <kern/monitor.h>

#include "../can/can_messages.h"

#include "stm32lib/stm32f10x_tim.h"

#define unlikely(x) __builtin_expect(!!(x), 0)

#define PRESCALER_VALUE 300

#define N_SMOOTH 5

/**
 * Initialise the beacon process.
 * Returns an event representing a beacon update.
 */
void beacon_init(void);

/**
 * Return the beacon positions.
 * The packets are valid when the function returns 0.
 */
int get_beacon_positions(beacon_position *pos, beacon_lowlevel_position *pos_ll);

#endif
