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

#include "battery_monitoring.h"

PROC_DEFINE_STACK(stack_battery_monitoring, KERN_MINSTACKSIZE * 16);

static battery_monitoring_ctx _ctx;
battery_monitoring_ctx *ctx = &_ctx;

static uint16_t battery_state[8];
static uint8_t measure_flag = 0;

int get_battery_monitoring(battery_status *pkt1, battery_status *pkt2) {
    uint8_t ch;

    if (measure_flag) {
        // Battery 1
        for (ch = 0; ch < 4; ++ch)
            pkt1->p.elem[ch] = battery_state[ch];

        // Battery 2
        for (ch = 0; ch < 4; ++ch)
            pkt2->p.elem[ch] = battery_state[ch];

        measure_flag = 0;
        return 0;
    }
    else
        return -1;

}

static void NORETURN battery_monitoring_process(void) {

    uint8_t battery_charge;

    int dead_cells = 0;
    int dead_cell_counter = 0;
    int low_charge_counter = 0;

    uint8_t ch;
    uint16_t value = 42;

    for (;;) {
        battery_charge = 2;

        // Battery 1
        for (ch = 0; ch < 4; ++ch) {
            value = adc7828_measure(ctx->i2c, ADS7828_ADDR_BASE, ch);
            battery_state[ch] = (uint16_t)(2.0*value*ADS7828_LSB * 10000.);

            if (battery_state[ch] < 10*LOW_CHARGE_THRES && battery_charge > 1)
                battery_charge = 1;
            else if (battery_state[ch] < 10*ABSENT_CELL_THRES && battery_charge > 0) {
                dead_cells++;
                battery_charge = 0;
            }
        }

        // Battery 2
        for (ch = 0; ch < 4; ++ch) {
            value = adc7828_measure(ctx->i2c, ADS7828_ADDR_BASE + 2, ch);
            battery_state[4+ch] = (uint16_t)(2.0*value*ADS7828_LSB * 10000.);

            if (battery_state[4+ch] < 10*LOW_CHARGE_THRES && battery_charge > 1)
                battery_charge = 1;
            else if (battery_state[4+ch] < 10*ABSENT_CELL_THRES && battery_charge > 0) {
                dead_cells++;
                battery_charge = 0;
            }
        }

        measure_flag = 1;

        if (battery_charge == 2) {
            // Full charge state
            set_buzzer(0);
        }
        else if (battery_charge == 1) {
            // Low charge state
            if (low_charge_counter < 100) {
                for (int j = 0; j < 4; j++) {
                    set_buzzer((j+1) % 2);
                    timer_delay(50);
                }
                low_charge_counter++;
            } else {
                set_buzzer(1);
            }
        }
        else if (battery_charge == 0 && dead_cell_counter < 2) {
            // No cell, or dead cell, or bad connection
            for (int j = 0; j < 6; j++) {
                set_buzzer((j+1) % 2);
                timer_delay(50);
            }
            dead_cell_counter++;
        }


        timer_delay(1000);
    }
}

battery_monitoring_ctx *battery_monitoring_init(I2c *i2c) {
    ctx->i2c = i2c;

    proc_new(battery_monitoring_process, NULL, sizeof(stack_battery_monitoring), stack_battery_monitoring);


    return ctx;
}
