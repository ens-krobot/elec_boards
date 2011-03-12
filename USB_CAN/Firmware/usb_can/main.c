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
#include <drv/can_stm32.h>
#include <drv/timer.h>

#include <kern/monitor.h>
#include <kern/proc.h>

PROC_DEFINE_STACK(stack1, KERN_MINSTACKSIZE * 8);
PROC_DEFINE_STACK(stack2, KERN_MINSTACKSIZE * 8);

static void init(void)
{

    can_config cfg;

    cfg.mcr = CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP;
    /* CAN driver in loopback mode */
    cfg.btr = CAN_BTR_SJW(0) | CAN_BTR_TS1(8) | CAN_BTR_TS2(1) | CAN_BTR_BRP(6) | CAN_BTR_LBKM;

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

	/*
	 * Kernel initialization: processes (allow to create and dispatch
	 * processes using proc_new()).
	 */
	proc_init();
}

static void NORETURN send_process(void)
{
    bool sent = 0;

    can_tx_frame txm;

    txm.dlc = 8;
    txm.rtr = 0;
    txm.ide = 1;
    txm.sid = 0;
    txm.eid = 0;
    txm.data32[0] = 0x42424242;
    txm.data32[1] = 0x42424242;

    for (;;) {
        txm.data32[0] = (txm.data32[0] << 4) | (txm.data32[0] & 0xf0000000) >> 28;
        txm.data32[1] = (txm.data32[1] << 2) | (txm.data32[1] & 0xc0000000) >> 30;
        txm.eid += 1;
        sent = can_transmit(CAND1, &txm, ms_to_ticks(100));
        /*        if (sent)
            kprintf("sent something... %d %08lX %08lX\n", txm.eid, txm.data32[0], txm.data32[1]);
        else
        kprintf("sent nothing...\n");*/

        if (txm.eid % 100 > 50)
            LED1_ON();
        else
            LED1_OFF();

        timer_delayHp(100);
    }
}

static void NORETURN receive_process(void)
{
    can_rx_frame rxm;
    uint32_t received = 0;

    for (;;) {
        can_receive(CAND1, &rxm, ms_to_ticks(100));
        received++;
        if (received % 1000 == 0)
            kprintf("received something... %d %08lx %08lx\n", rxm.eid, rxm.data32[0], rxm.data32[1]);
    }
}

int main(void)
{

    /* Hardware initialization */
    init();
    
    /* Create a new child process */
    proc_new(send_process, NULL, sizeof(stack1), stack1);
    proc_new(receive_process, NULL, sizeof(stack2), stack2);

    for (;;) {
        monitor_report();
        timer_delay(1000);
    }

}
