/**
 * AX12 Library
 *
 * This file contains the interface for AX12 communication
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

#ifndef AX12_H__
#define AX12_H__

#include <stdlib.h>
#include <stdint.h>

#include "serial.h"

#define KROBOT_AX12_BAUDRATE 115200

/**
 * AX12 Commands
 */
typedef enum {
    AX12_CMD_PING = 0x01,
    AX12_CMD_READ_DATA = 0x02,
    AX12_CMD_WRITE_DATA = 0x03,
    AX12_CMD_REG_WRITE = 0x04,
    AX12_CMD_ACTION = 0x05,
    AX12_CMD_RESET = 0x06,
    AX12_CMD_SYNC_WRITE = 0x83
} ax12_cmd;

/**
 * AX12 Memory Map
 */
typedef enum {
    AX12_MODEL = 0x00,
    AX12_VERSION = 0x02,
    AX12_ID = 0x03,
    AX12_BAUDRATE = 0x04,
    AX12_RETURN_DELAY = 0x05,
    AX12_CW_ANGLE_LIMIT = 0x06,
    AX12_CCW_ANGLE_LIMIT = 0x08,
    AX12_LIMIT_TEMPERATURE_HI = 0x0B,
    AX12_LIMIT_VOLTAGE_LO = 0x0C,
    AX12_LIMIT_VOLTAGE_HI = 0x0D,
    AX12_MAX_TORQUE = 0x0E,
    AX12_STATUS_RETURN_LEVEL = 0x10,
    AX12_ALARM_LED = 0x11,
    AX12_ALARM_SHUTDOWN = 0x12,
    AX12_DOWN_CALIBRATION = 0x14,
    AX12_UP_CALIBRATION = 0x16,
    AX12_TORQUE_ENABLE = 0x18,
    AX12_LED = 0x19,
    AX12_CW_COMPLIANCE_MARGIN = 0x1A,
    AX12_CCW_COMPLIANCE_MARGIN = 0x1B,
    AX12_CW_COMPLIANCE_SLOPE = 0x1C,
    AX12_CCW_COMPLIANCE_SLOPE = 0x1D,
    AX12_GOAL_POSITION = 0x1E,
    AX12_MOVING_SPEED = 0x20,
    AX12_TORQUE_LIMIT = 0x22,
    AX12_PRESENT_POSITION = 0x24,
    AX12_PRESENT_SPEED = 0x26,
    AX12_PRESENT_LOAD = 0x28,
    AX12_PRESENT_VOLTAGE = 0x2A,
    AX12_PRESENT_TEMPERATURE = 0x2B,
    AX12_MOVING = 0x2E,
    AX12_LOCK = 0x2F,
    AX12_PUNCH = 0x30
} ax12_memory_map;

#define AX12_BROADCAST 0xFE

/**
 * An AX12 command packet
 */
typedef struct {
    uint8_t address;
    ax12_cmd command;
    uint8_t length;
    uint8_t *args;
} ax12_cmd_packet;

/**
 * An AX12 error bitfield
 */
typedef struct {
    uint8_t :1;
    uint8_t instruction:1;
    uint8_t overload:1;
    uint8_t checksum:1;
    uint8_t range:1;
    uint8_t overheating:1;
    uint8_t angle:1;
    uint8_t voltage:1;
} ax12_error;

/**
 * An AX12 status packet
 */
typedef struct {
    uint8_t address;
    union {
        uint8_t i;
        ax12_error e;
    } error;
    uint8_t length;
    uint8_t *args;
} ax12_st_packet;

void ax12_write(ax12_cmd_packet *pkt);
int ax12_read(ax12_st_packet *pkt);
int ax12_reset(uint8_t new_id);

#endif
