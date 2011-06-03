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

#include "ax12_highlevel.h"

#define N_QUEUED 16

static struct ax12_hl_command queued_commands[N_QUEUED];
static int queued_command_read = 0;
static int queued_command_write = 0;

static ax12_state queued_responses[N_QUEUED];
static int queued_response_read = 0;
static int queued_response_write = 0;


PROC_DEFINE_STACK(ax12_stack, KERN_MINSTACKSIZE * 16);
static void NORETURN ax12_process(void);

void ax12_highlevel_init(void) {
    serial_init(KROBOT_AX12_BAUDRATE);
    proc_new(ax12_process, NULL, sizeof(ax12_stack), ax12_stack);
}

static void ax12_goto_hl(uint8_t address, uint16_t position, uint16_t speed) {

    ax12_cmd_packet pkt;

    uint8_t args[5];

    if (position > 0x3ff || speed > 0x3ff)
        return;

    pkt.address = address;
    pkt.command = AX12_CMD_WRITE_DATA;
    pkt.length = 5;
    pkt.args = args;
    pkt.args[0] = AX12_GOAL_POSITION;
    pkt.args[1] = position & 0xFF;
    pkt.args[2] = (position >> 8) & 0xFF;
    pkt.args[3] = speed & 0xFF;
    pkt.args[4] = (speed >> 8) & 0xFF;

    ax12_write(&pkt);
    ax12_read(NULL);
}

static void ax12_get_state_hl(uint8_t address) {

    ax12_cmd_packet pkt;
    ax12_st_packet status;

    ax12_state *state = queued_responses + (queued_response_write % N_QUEUED);

    uint8_t args[3];
    uint8_t recv_args[6];

    int i = 0;

    status.args = recv_args;

    pkt.address = address;
    pkt.command = AX12_CMD_READ_DATA;
    pkt.length = 2;
    pkt.args = args;
    pkt.args[0] = AX12_PRESENT_POSITION;
    pkt.args[1] = 6;

    ax12_write(&pkt);
    while (ax12_read(&status) < 0 || status.address != address) {
        if (i >= 10)
            return;
        timer_delay(10);
        i++;
    }

    state->p.address = status.address;
    state->p.position = status.args[1] << 8 | status.args[0];
    state->p.speed = status.args[3] << 8 | status.args[2];
    state->p.torque = status.args[5] << 8 | status.args[4];

    queued_response_write++;
}

static void ax12_set_torque_enable_hl(uint8_t address, uint8_t enable) {
    ax12_cmd_packet pkt;
    uint8_t args[3];

    pkt.address = address;
    pkt.command = AX12_CMD_WRITE_DATA;
    pkt.length = 2;
    pkt.args = args;
    pkt.args[0] = AX12_TORQUE_ENABLE;
    pkt.args[1] = enable == 0 ? 0 : 1;

    ax12_write(&pkt);
    ax12_read(NULL);

}

static void NORETURN ax12_process(void) {

    struct ax12_hl_command *read = NULL;

    for (;;) {
        while (queued_command_read == queued_command_write)
            cpu_relax();

        read = queued_commands + (queued_command_read % N_QUEUED);

        switch (read->command) {
          case AX12_HL_GET_STATE:
            ax12_get_state_hl(read->address);
            break;
          case AX12_HL_GOTO:
            ax12_goto_hl(read->address, read->args[0], read->args[1]);
            break;
          case AX12_SET_TORQUE_ENABLE:
            ax12_set_torque_enable_hl(read->address, read->args[0]);
            break;
        }

        queued_command_read++;
    }

}

void ax12_queue_command(struct ax12_hl_command *cmd) {
    struct ax12_hl_command *queued_command;

    queued_command = queued_commands + (queued_command_write % N_QUEUED);

    queued_command->command = cmd->command;
    queued_command->address = cmd->address;
    queued_command->args[0] = cmd->args[0];
    queued_command->args[1] = cmd->args[1];

    queued_command_write++;
}

int ax12_dequeue_state(ax12_state *state) {

    ax12_state *cur_read_state;

    if (queued_response_read == queued_response_write)
        return -1;

    cur_read_state = queued_responses + (queued_response_read % N_QUEUED);

    state->p.address = cur_read_state->p.address;
    state->p.position = cur_read_state->p.position;
    state->p.speed = cur_read_state->p.speed;
    state->p.torque = cur_read_state->p.torque;

    queued_response_read++;

    return 0;
}
