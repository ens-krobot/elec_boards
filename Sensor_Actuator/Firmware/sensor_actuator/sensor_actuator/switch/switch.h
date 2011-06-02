/**
 * Sensor-Actuator board : Switches
 *
 * Header file for the switches management.
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
#ifndef SWITCH_H
#define SWITCH_H

#include <drv/clock_stm32.h>
#include <drv/gpio_stm32.h>

#include "../can/can_messages.h"

/**
 * Initialise the switches
 */
void switch_init(void);

/**
 * Get the switch CAN packet
 */
void get_switch_status(switch_status *pkt1, switch_status *pkt2);

/**
 * Set the switch state from the CAN packet
 */
void set_switch(switch_request *pkt);

/**
 * Set the buzzer state
 */
void set_buzzer(uint8_t state);

#endif
