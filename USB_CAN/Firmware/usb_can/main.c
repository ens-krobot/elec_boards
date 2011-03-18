/**
 * \file
 * <!--
 * This file is part of BeRTOS.
 *
 * Bertos is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * As a special exception, you may use this file as part of a free software
 * library without restriction.  Specifically, if other files instantiate
 * templates or use macros or inline functions from this file, or you compile
 * this file and link it with other files to produce an executable, this
 * file does not by itself cause the resulting executable to be covered by
 * the GNU General Public License.  This exception does not however
 * invalidate any other reasons why the executable file might be covered by
 * the GNU General Public License.
 *
 * Copyright 2010 Develer S.r.l. (http://www.develer.com/)
 *
 * -->
 *
 * \author Andrea Righi <arighi@develer.com>
 *
 * \brief Kernel project.
 *
 * This is a minimalist kernel project: it just initializes the hardware and
 * creates an independent process to blink an LED, while the main loop
 * continues to monitor the stack utilization of all the processes.
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

static void init(void)
{

    can_config cfg;

    cfg.mcr = CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP;
    /* CAN driver in loopback mode */
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

    /*
     * Kernel initialization: processes (allow to create and dispatch
     * processes using proc_new()).
     */
    proc_init();
}

static void NORETURN serial_receive_process(void)
{
    int nbytes, retval, i = 0;
    char command[MAX_CMD_SIZE+1];

    can_tx_frame frame;

    frame.ide = 1;
    frame.eid = 0x42;
    frame.dlc = 4;
    frame.data8[0] = 1;
    frame.data8[1] = 1;
    frame.data8[2] = 1;
    frame.data8[3] = 1;

    for (;;) {
        i = !i;
        nbytes = kfile_gets(&ser.fd, command, MAX_CMD_SIZE+1);
        if (nbytes != EOF) {
            retval = usb_can_execute_command(CAND1, &ser, command);
            can_transmit(CAND1, &frame, ms_to_ticks(42));
            kprintf("got %d bytes: [%s]\n", nbytes, command);
            if (i)
                LED1_ON();
            else
                LED1_OFF();
        } else {
            kprintf("got EOF :(\n");
        }
    }
}

static void NORETURN can_receive_process(void) {

    can_rx_frame frame;
    int retval;
    int32_t timestamp;

    for (;;) {
        can_receive(CAND1, &frame, ms_to_ticks(100));
        timestamp = ticks_to_ms(timer_clock()) % 60000;
        retval = usb_can_emit(CAND1, &ser, &frame, (uint16_t)timestamp);
        kprintf("received something... %d %08lx %08lx\n", frame.ide ? frame.eid:frame.sid, frame.data32[0], frame.data32[1]);
    }
}

static void NORETURN blinky_process(void) {
    for (;;) {
        LED2_ON();
        timer_delay(500);
        LED2_OFF();
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
