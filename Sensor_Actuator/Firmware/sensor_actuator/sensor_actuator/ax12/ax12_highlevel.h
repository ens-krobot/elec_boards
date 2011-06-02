/**
 * AX12 High-Level Library
 *
 * This file contains the high-level interface for AX12 communication
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

#ifndef AX12_HILEVEL_H__
#define AX12_HILEVEL_H__

#include <stdint.h>

#include <cpu/power.h>
#include <kern/proc.h>

#include "../can/can_messages.h"

#include "ax12.h"
#include "serial.h"

struct ax12_hl_command {
    enum {
        AX12_HL_GET_STATE,
        AX12_HL_GOTO,
    } command;
    uint8_t address;
    uint16_t args[2];
};

void ax12_highlevel_init(void);

void ax12_queue_command(struct ax12_hl_command *cmd);
int ax12_dequeue_state(ax12_state *state);

#endif
