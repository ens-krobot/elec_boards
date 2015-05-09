/**
 * Sensor and Actuator interface board
 *
 * This file is the entry point for the Sensor_Actuator firmware
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

#include <drv/clock_stm32.h>
#include <drv/gpio_stm32.h>

#include <drv/can.h>
#include <drv/timer.h>
#include <drv/i2c.h>

#include <kern/monitor.h>
#include <kern/proc.h>
#include <io/kfile.h>

#include "adc/adc.h"
#include "ax12/ax12_highlevel.h"
#include "battery_monitoring/battery_monitoring.h"
#include "beacon/beacon.h"
#include "can/can_monitor.h"
#include "switch/switch.h"
#include "lcd/lcd.h"

PROC_DEFINE_STACK(stack_blinky, KERN_MINSTACKSIZE * 2);

static I2c i2c1, i2c2;

static void init(void)
{
    /* Enable all the interrupts */
    IRQ_ENABLE;

    /* Initialize I2c interface */
    i2c_init(&i2c1, 0, CONFIG_I2C_FREQ);
    i2c_init(&i2c2, 1, CONFIG_I2C_FREQ);

    /* Initialize system timer */
    timer_init();

    /* Initialize LED driver */
    LEDS_INIT();

    LED_ON();

    /*
     * Kernel initialization: processes (allow to create and dispatch
     * processes using proc_new()).
     */
    proc_init();

    // Initialize the LCD screen
    lcd_init(&i2c1, 0x0E);
    lcd_cls();
    lcd_set_cursor(DEACTIVE);
    lcd_write_line("     == Nuky ==     ", 1);
    lcd_write_line("         by [Kro]bot", 2);

    // Initialize the rotary beacon
    beacon_init();

    // Initialize the switches
    switch_init();

    // Initialize the ADCs
    sa_adc_init();

    // Initialize the Battery monitoring
    battery_monitoring_init(&i2c2, 3);

    // Initialize the AX12s
    ax12_highlevel_init();

    // Initialize the CAN bus processing
    can_processes_init();
}

static void NORETURN blinky_process(void) {
    for (;;) {
      //LED_ON();
      //lcd_set_backlight(ACTIVE);
      timer_delay(1000);
      //lcd_set_backlight(DEACTIVE);
      //LED_OFF();
      timer_delay(1000);
    }
}

int main(void)
{
/*
    int i = 0;
    uint8_t t = 0;

    switch_request r;
*/
    /* Hardware initialization */
    init();

    /* Create a new child process */
    proc_new(blinky_process, NULL, sizeof(stack_blinky), stack_blinky);

    for (;;) {
/*
        r.p.num = i + 5;
        r.p.state = t;

        set_switch(&r);
*/
        timer_delay(1000);
/*
        i = (i+1) % 3;
        t = t?0:1;
*/
    }

}
