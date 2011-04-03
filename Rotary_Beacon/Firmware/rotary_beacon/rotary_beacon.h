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
#include <io/stm32_gpio.h>
#include <mware/event.h>
#include <kern/monitor.h>

#include "stm32lib/stm32f10x_tim.h"

#define unlikely(x) __builtin_expect(!!(x), 0)

#define PRESCALER_VALUE 300

#define N_SMOOTH 5

// Data packets

struct beacon_position_pkt {
    uint16_t angle __attribute__((packed));    // in 1/10000th of radians [0; 2*Pi[
    uint16_t distance __attribute__((packed)); // in mm [0; 65536[
    uint16_t period __attribute__((packed));   // in 1/10000th of seconds [0; 1[
};

typedef union {
    struct beacon_position_pkt p;
    uint32_t d[2];
} beacon_position;

struct beacon_lowlevel_position_pkt {
    uint16_t angle __attribute__((packed)); // in 1/10000th of radians [0; 2*Pi[
    uint16_t width __attribute__((packed)); // in 1/100000th of radians [0; Pi/5[
    uint32_t period __attribute__((packed)); // in timer ticks
};

typedef union {
    struct beacon_lowlevel_position_pkt p;
    uint32_t d[2];
} beacon_lowlevel_position;

struct beacon_calibration_pkt {
    uint16_t width __attribute__((packed)); // in 1/100000th of radians [0; Pi/5[
    uint16_t distance __attribute__((packed)); // in mm [0; 65536[
};

typedef union {
    struct beacon_calibration_pkt p;
    uint32_t d[2];
} beacon_calibration;

void beacon_init(void);
#endif
