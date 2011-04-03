/**
 * Rotary beacon
 *
 * This file is the entry point for the rotary beacon firmware.
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

#include "hw/hw_led.h"

#include <cfg/debug.h>

#include <cpu/irq.h>

#include <drv/can.h>
#include <drv/timer.h>

#include <kern/proc.h>
#include <kern/monitor.h>

#include "rotary_beacon.h"

static void init(void)
{

    can_config cfg;

    // Enable all the interrupts
    IRQ_ENABLE;

    // Initialize debugging module (allow kprintf(), etc.)
    kdbg_init();
    // Initialize system timer
    timer_init();
    // Initialize LED driver
    LED_INIT();

    // CAN Bus configuration
    cfg.mcr = CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP;
    /* 1 Mbit by default (FIXME: O'RLY?) */
    cfg.btr = CAN_BTR_SJW(0) | CAN_BTR_TS1(8) | CAN_BTR_TS2(1) | CAN_BTR_BRP(6);

    cfg.n_filters = 0;
    cfg.filters = NULL;

    can_init();

    can_start(CAND1, &cfg);

    /*
     * Kernel initialization: processes (allow to create and dispatch
     * processes using proc_new()).
     */
    proc_init();

    // Initialize Beacon driver
    beacon_init();

}

int main(void)
{
    /* Hardware initialization */
    init();

    /*
     * The main process is kept to periodically report the stack
     * utilization of all the processes (1 probe per second).
     */
    while (1) {
        //monitor_report();
        timer_delay(1000);
    }
}
