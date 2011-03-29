/**
 * USB-CAN converter
 *
 * This file is the entry point for an USB <-> CAN transceiver.
 *
 * Copyright © 2011 Nicolas Dandrimont <olasd@crans.org>
 * Authors: Nicolas Dandrimont <olasd@crans.org>
 *          Xavier Lagorce <Xavier.Lagorce@crans.org>
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

#include <drv/gpio_stm32.h>

#include <drv/can.h>
#include <drv/ser.h>
#include <drv/timer.h>

#include <kern/monitor.h>
#include <kern/proc.h>
#include <io/kfile.h>

#include "usb_can.h"

#define MAX_CMD_SIZE 64

PROC_DEFINE_STACK(stack_ser_recv, KERN_MINSTACKSIZE * 4);
PROC_DEFINE_STACK(stack_can_recv, KERN_MINSTACKSIZE * 4);
PROC_DEFINE_STACK(stack_blinky, KERN_MINSTACKSIZE * 2);

static struct Serial ser;
static usb_can usbcan;

static void init(void)
{
    can_config cfg;

    cfg.mcr = CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP;
    /* 1 Mbit by default (FIXME: O'RLY?) */
    cfg.btr = CAN_BTR_SJW(0) | CAN_BTR_TS1(8) | CAN_BTR_TS2(1) | CAN_BTR_BRP(6);

    cfg.n_filters = 0;
    cfg.filters = NULL;


    /* Enable all the interrupts */
    IRQ_ENABLE;

    /* Initialize debugging module (allow kprintf(), etc.) */
    kdbg_init();
    /* Initialize system timer */
    timer_init();


    /* Initialize CAN driver */
    can_init();

    can_start(CAND1, &cfg);

    /* Initialize LED driver */
    LEDS_INIT();

    LED3_ON();

    /* Initialize Serial driver */
    ser_init(&ser, SER_UART3);

    /* Initialize USB-CAN logic */
    usb_can_init(&usbcan, CAND1, &ser);

    /*
     * Kernel initialization: processes (allow to create and dispatch
     * processes using proc_new()).
     */
    proc_init();
}

static void NORETURN serial_receive_process(void)
{
    int nbytes = 0, retval, i = 0;
    char command[MAX_CMD_SIZE+1];

    for (;;) {
        i = !i;
        nbytes = kfile_gets(&usbcan.ser->fd, command, MAX_CMD_SIZE+1);
        if (nbytes != EOF) {
            retval = usb_can_execute_command(&usbcan, command);
            if (i)
                LED1_ON();
            else
                LED1_OFF();
        }
    }
}

static void NORETURN can_receive_process(void) {

    can_rx_frame frame;
    int retval;
    bool received = false;
    bool i = false;


    for (;;) {
        received = can_receive(usbcan.can, &frame, ms_to_ticks(100));
        if (received) {
            i = !i;
            retval = usb_can_emit(&usbcan, &frame);
            if (i)
                LED2_ON();
            else
                LED2_OFF();
        }
    }
}

static void NORETURN blinky_process(void) {
    for (;;) {
        LED3_ON();
        timer_delay(500);
        LED3_OFF();
        timer_delay(500);
    }
}

int main(void)
{

    /* Hardware initialization */
    init();

    /* Create a new child process */
    proc_new(serial_receive_process, NULL, sizeof(stack_ser_recv), stack_ser_recv);
    proc_new(can_receive_process, NULL, sizeof(stack_can_recv), stack_can_recv);
    proc_new(blinky_process, NULL, sizeof(stack_blinky), stack_blinky);

    for (;;) {
        timer_delay(1000);
    }

}
