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

#include "../usb_can/usb_can.h"
#include "ads7828.h"

typedef struct {
    usb_can *usbcan;
    I2c *i2c;
} battery_monitoring_ctx;

struct battery_status_pkt {
    uint16_t elem[4] __attribute__((packed)); // in 1/10000th volts [0; 6.5536[
};

typedef union {
    struct battery_status_pkt p;
    uint32_t d[2];
} battery_status;

battery_monitoring_ctx *battery_monitoring_init(usb_can *usbcan, I2c *i2c);

#endif
