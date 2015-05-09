/**
 * Battery Monitoring wrapper for the USB CAN Board
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

#ifndef BATTERY_MONITORING_H
#define BATTERY_MONITORING_H

#include <drv/can.h>
#include <drv/i2c.h>
#include <drv/timer.h>

#include "../can/can_messages.h"
#include "../switch/switch.h"

#include "ads7828.h"

#define LOW_CHARGE_THRES        2800                ///< Low charge threshold (in mV) -- trigger an alarm
#define ABSENT_CELL_THRES        100                ///< Threshold for no cell connected (or dead cell, or bad connection)

typedef struct {
    I2c *i2c;
  unsigned char num_elem;
} battery_monitoring_ctx;

battery_monitoring_ctx *battery_monitoring_init(I2c *i2c, unsigned char num_elem);

int get_battery_monitoring(battery_status *pkt1, battery_status *pkt2);

#endif
