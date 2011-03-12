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
 * Copyright © 2011 Nicolas Dandrimont <Nicolas.Dandrimont@crans.org>
 *
 * -->
 *
 * \brief CAN driver (implementation)
 *
 * \author Nicolas Dandrimont <Nicolas.Dandrimont@crans.org>
 */

#include <drv/can.h>

#ifndef WIZ_AUTOGEN
	#warning Deprecated: now you should include can_<cpu> directly in the makefile. Remove this line and the following once done.
	#include CPU_CSOURCE(can)
#else
	#include CPU_HEADER(can)
#endif

#include <cfg/debug.h>     // ASSERT()
#include <cfg/log.h>
#include <cfg/macros.h>    // MIN()
#include <cfg/compiler.h>
#include <cfg/module.h>

MOD_DEFINE(can);

/**
 * Initialize the CAN hardware.
 */
void can_init(void)
{
	can_hw_init();

	MOD_INIT(can);
}

/**
 * Initialize the default fields for CAN driver \a drv
 */
void can_drv_init(can_driver *drv)
{
    drv->state = CAN_STOP;
    drv->config = NULL;
    drv->rx_available_event = event_createGeneric();
    drv->tx_empty_event = event_createGeneric();
    drv->sleep_event = event_createGeneric();
    drv->wakeup_event = event_createGeneric();
    drv->error_event = event_createGeneric();
    drv->error_flags = 0;
}

/**
 * Configure and start the CAN peripheral
 */
void can_start(can_driver *drv, const can_config *config) {

    proc_forbid();
    while (drv->state == CAN_STARTING)
        timer_delayTicks(1);
    if (drv->state == CAN_STOP) {
        drv->config = config;
        can_hw_start(drv);
        drv->state = CAN_READY;
    }
    proc_permit();
}

/**
 * Deactivate the CAN peripheral
 */
void can_stop(can_driver *drv) {

    proc_forbid();
    can_hw_stop(drv);
    drv->state = CAN_STOP;
    drv->error_flags = 0;
    proc_permit();
}

/**
 * Transmit a CAN frame
 */
bool can_transmit(can_driver *drv, can_tx_frame *frame, ticks_t timeout) {

    bool ret;

    proc_forbid();
    while ((drv->state == CAN_SLEEP) || !can_hw_can_transmit(drv)) {
        proc_permit();
        cpu_relax();
        proc_forbid();
    }

    can_hw_transmit(drv, frame);
    proc_permit();
    return true;
}

/**
 * Receive a CAN frame
 */
bool can_receive(can_driver *drv, can_rx_frame *frame, ticks_t timeout) {

    bool ret;

    proc_forbid();
    while ((drv->state == CAN_SLEEP) || !can_hw_can_receive(drv)) {
        proc_permit();
        cpu_relax();
        proc_forbid();
    }

    can_hw_receive(drv, frame);
    proc_permit();
    return true;
}

/**
 * Return the status mask and clear it
 */
can_errorflags can_get_and_clear_flags(can_driver *drv) {
    can_errorflags errorflags;

    proc_forbid();
    errorflags = drv->error_flags;
    drv->error_flags = 0;
    proc_permit();

    return errorflags;
}

/**
 * Put the CAN device into sleep mode
 */
void can_sleep(can_driver *drv) {

    proc_forbid();
    if (drv->state == CAN_READY) {
        can_hw_sleep(drv);
        drv->state = CAN_SLEEP;
        event_do(&drv->sleep_event);
    }
    proc_permit();

}

/**
 * Wake the CAN device up
 */
void can_wakeup(can_driver *drv) {

    proc_forbid();
    if (drv->state == CAN_SLEEP) {
        can_hw_wakeup(drv);
        drv->state = CAN_READY;
        event_do(&drv->wakeup_event);
    }
    proc_permit();
}
