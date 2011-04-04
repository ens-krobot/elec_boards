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

static void NORETURN battery_monitoring_process(void) {

    can_tx_frame txf;
    can_rx_frame rxf;

    battery_status b;

    uint8_t ch;
    uint16_t value = 42;

    txf.ide = rxf.ide = 1;
    txf.rtr = rxf.rtr = 0;
    txf.dlc = rxf.dlc = sizeof(b.p);

    for (;;) {
        // Battery 1
        for (ch = 0; ch < 4; ++ch) {
            value = adc7828_measure(ctx->i2c, ADS7828_ADDR_BASE, ch);
            b.p.elem[ch] = (uint16_t)(2.0*value*ADS7828_LSB * 10000.);
        }

        txf.eid = rxf.eid = 401;
        txf.data32[0] = rxf.data32[0] = b.d[0];
        txf.data32[1] = rxf.data32[1] = b.d[1];

        usb_can_emit(ctx->usbcan, &rxf);
        can_transmit(ctx->usbcan->can, &txf, ms_to_ticks(10));

        // Battery 2
        for (ch = 0; ch < 4; ++ch) {
            value = adc7828_measure(ctx->i2c, ADS7828_ADDR_BASE + 2, ch);
            b.p.elem[ch] = (uint16_t)(2.0*value*ADS7828_LSB * 10000.);
        }

        txf.eid = rxf.eid = 402;
        txf.data32[0] = rxf.data32[0] = b.d[0];
        txf.data32[1] = rxf.data32[1] = b.d[1];

        usb_can_emit(ctx->usbcan, &rxf);
        can_transmit(ctx->usbcan->can, &txf, ms_to_ticks(10));

        timer_delay(100);
    }
}

battery_monitoring_ctx *battery_monitoring_init(usb_can *usbcan, I2c *i2c) {
    ctx->usbcan = usbcan;
    ctx->i2c = i2c;

    proc_new(battery_monitoring_process, NULL, sizeof(stack_battery_monitoring), stack_battery_monitoring);


    return ctx;
}
