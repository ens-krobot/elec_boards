/**
 * Sensor-Actuator board CAN processes
 *
 * Header file for the Sensor and Actuator CAN processes initialisation
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

#ifndef CAN_MONITOR_H__
#define CAN_MONITOR_H__

#include <drv/can.h>
#include <drv/timer.h>

#include <kern/proc.h>

#include "can_messages.h"

#include "../adc/adc.h"
#include "../lcd/lcd.h"
#include "../ax12/ax12_highlevel.h"
#include "../beacon/beacon.h"
#include "../switch/switch.h"
#include "../battery_monitoring/battery_monitoring.h"

void can_processes_init(void);

#endif
