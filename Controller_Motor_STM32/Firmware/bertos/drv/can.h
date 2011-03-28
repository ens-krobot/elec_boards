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
 * \defgroup can Generic CAN driver
 * \ingroup drivers
 * \{
 * \brief Controller Area Network driver (CAN).
 *
 * <b>Configuration file</b>: cfg_can.h
 *
 * \author Nicolas Dandrimont <Nicolas.Dandrimont@crans.org>
 *
 * $WIZ$ module_name = "can"
 * $WIZ$ module_configuration = "bertos/cfg/cfg_can.h"
 * $WIZ$ module_supports = "stm32"
 * $WIZ$ module_depends = "timer", "signal"
 *
 */


#ifndef DRV_CAN_H
#define DRV_CAN_H

#include <cfg/compiler.h>
#include <cfg/debug.h>
#include <cpu/attr.h>
#include CPU_HEADER(can)

void can_init(void);
void can_drv_init(can_driver *drv);
void can_start(can_driver *drv, const can_config *config);
void can_stop(can_driver *drv);
bool can_transmit(can_driver *drv, can_tx_frame *frame, ticks_t timeout);
bool can_receive(can_driver *drv, can_rx_frame *frame, ticks_t timeout);
can_errorflags can_get_and_clear_flags(can_driver *drv);
void can_sleep(can_driver *drv);
void can_wakeup(can_driver *drv);
/** \} */ //defgroup can
#endif /* DRV_CAN_H */
