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

#include <drv/clock_stm32.h>
#include <drv/gpio_stm32.h>

#include <drv/can.h>
#include <drv/i2c.h>
#include <drv/ser.h>
#include <drv/timer.h>

#include <kern/monitor.h>
#include <kern/proc.h>
#include <io/kfile.h>

#include "battery_monitoring/battery_monitoring.h"
#include "usb_can/usb_can.h"

#define SERIAL_BAUDRATE 1000000

PROC_DEFINE_STACK(stack_blinky, KERN_MINSTACKSIZE * 2);

static struct Serial ser;

static I2c i2c;

static void init(void)
{
    can_config cfg;

    usb_can *usbcan;

    cfg.mcr = CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP;
    /* 1 Mbit by default (FIXME: O'RLY?) */
    cfg.btr = CAN_BTR_SJW(0) | CAN_BTR_TS1(8) | CAN_BTR_TS2(1) | CAN_BTR_BRP(6);

    cfg.n_filters = 0;
    cfg.filters = NULL;


    /* Enable all the interrupts */
    IRQ_ENABLE;

    //RCC->APB2ENR |= RCC_APB2_AFIO;
    //stm32_gpioRemap(GPIO_REMAP_USART1, GPIO_REMAP_ENABLE);

    /* Initialize debugging module (allow kprintf(), etc.) */
    //kdbg_init();
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
    ser_setbaudrate(&ser, SERIAL_BAUDRATE);

    /* Initialize I2c interface */
    i2c_init(&i2c, 0, CONFIG_I2C_FREQ);

    /*
     * Kernel initialization: processes (allow to create and dispatch
     * processes using proc_new()).
     */
    proc_init();

    /* Initialize USB-CAN logic */
    usbcan = usb_can_init(CAND1, &ser);

    /* Initialize battery monitoring */
    battery_monitoring_init(usbcan, &i2c);
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
    proc_new(blinky_process, NULL, sizeof(stack_blinky), stack_blinky);

    for (;;) {
        timer_delay(1000);
    }

}
