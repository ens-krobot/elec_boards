/**
 * Sensor-Actuator board CAN processes
 *
 * Header file for the Sensor and Actuator CAN processes initialisation
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

#include "can_monitor.h"

PROC_DEFINE_STACK(stack_send, KERN_MINSTACKSIZE * 4)
PROC_DEFINE_STACK(stack_recv, KERN_MINSTACKSIZE * 4)

static void NORETURN can_sender_process(void);
static void NORETURN can_receiver_process(void);

volatile uint8_t sim_mode = SIMULATION_MODE_NO;

void can_processes_init(void) {
    can_config cfg;

    cfg.mcr = CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP;
    /* 1 Mbit by default (FIXME: O'RLY?) */
    cfg.btr = CAN_BTR_SJW(0) | CAN_BTR_TS1(8) | CAN_BTR_TS2(1) | CAN_BTR_BRP(6);

    cfg.n_filters = 0;
    cfg.filters = NULL;

    /* Initialize CAN driver */
    can_init();

    can_start(CAND1, &cfg);

    proc_new(can_sender_process, NULL, sizeof(stack_send), stack_send);
    proc_new(can_receiver_process, NULL, sizeof(stack_recv), stack_recv);

}

#define SET_PACKET(f, id, pk) do {              \
        f.eid = id;                             \
        f.dlc = sizeof(pk.p);                   \
        f.data32[0] = pk.d[0];                  \
        f.data32[1] = pk.d[1];                  \
    } while (0);

static void NORETURN can_sender_process(void) {

    Timer timer_send;

    can_tx_frame f;

    beacon_position pos;
    beacon_lowlevel_position pos_ll;
    beacon_angles angles;
    beacon_widths widths;

    switch_status st1, st2;

    adc_values adc1, adc2;

    battery_status battery1, battery2;

    ax12_state ax12_st;

    int i = 0;

    /* Initialize can frame */

    f.ide = 1;
    f.rtr = 0;

    timer_setDelay(&timer_send, ms_to_ticks(50));
    timer_setEvent(&timer_send);

    for (;;) {
        timer_add(&timer_send);

        if (sim_mode == SIMULATION_MODE_NO) {
            /* Beacon */
            get_beacon_positions(&pos, &pos_ll, &angles, &widths);

            SET_PACKET(f, CAN_BEACON_POSITION, pos);
            can_transmit(CAND1, &f, ms_to_ticks(10));

            SET_PACKET(f, CAN_BEACON_LOWLEVEL_POSITION, pos_ll);
            can_transmit(CAND1, &f, ms_to_ticks(10));

            SET_PACKET(f, CAN_BEACON_ANGLES, angles);
            can_transmit(CAND1, &f, ms_to_ticks(10));

            SET_PACKET(f, CAN_BEACON_WIDTHS, widths);
            can_transmit(CAND1, &f, ms_to_ticks(10));

            /* Switches */

            get_switch_status(&st1, &st2);

            SET_PACKET(f, CAN_SWITCH_STATUS_1, st1);
            can_transmit(CAND1, &f, ms_to_ticks(10));

            SET_PACKET(f, CAN_SWITCH_STATUS_2, st2);
            can_transmit(CAND1, &f, ms_to_ticks(10));

            /* ADC */

            get_adc_values(&adc1, &adc2);

            SET_PACKET(f, CAN_ADC_VALUES_1, adc1);
            can_transmit(CAND1, &f, ms_to_ticks(10));

            SET_PACKET(f, CAN_ADC_VALUES_2, adc2);
            can_transmit(CAND1, &f, ms_to_ticks(10));

            /* Battery monitoring */

            if (get_battery_monitoring(&battery1, &battery2) == 0) {

                SET_PACKET(f, CAN_BATTERY_STATUS_1, battery1);
                can_transmit(CAND1, &f, ms_to_ticks(10));

                SET_PACKET(f, CAN_BATTERY_STATUS_2, battery2);
                can_transmit(CAND1, &f, ms_to_ticks(10));
            }

            /* AX-12 */

            while (ax12_dequeue_state(&ax12_st) == 0) {
                SET_PACKET(f, CAN_AX12_STATE, ax12_st);
                can_transmit(CAND1, &f, ms_to_ticks(10));
            }
        }

        if (i)
            LED_ON();
        else
            LED_OFF();

        i = !i;

        timer_waitEvent(&timer_send);
    }

}

#define GET_PACKET(type, name, frame) type name; name.d[0] = frame.data32[0]; name.d[1] = frame.data32[1]

static void NORETURN can_receiver_process(void) {

    can_rx_frame f;

    int ret;

    for (;;) {
        ret = can_receive(CAND1, &f, ms_to_ticks(200));
        if (!ret || f.ide != 1)
            continue;

        // These messages are to be interpreted only when not in simulation mode
        if (sim_mode == SIMULATION_MODE_NO) {
          switch (f.eid) {
          case CAN_SWITCH_SET:
            do {
              GET_PACKET(switch_request, req, f);
              set_switch(&req);
            } while (0);
            break;

          case CAN_AX12_REQUEST_STATE:
            do {
              struct ax12_hl_command hlc;
              GET_PACKET(ax12_request_state, ax12_req, f);
              hlc.command = AX12_HL_GET_STATE;
              hlc.address = ax12_req.p.address;
              ax12_queue_command(&hlc);
            } while (0);
            break;
          case CAN_AX12_GOTO:
            do {
              struct ax12_hl_command hlc;
              GET_PACKET(ax12_goto, ax12_g, f);
              hlc.command = AX12_HL_GOTO;
              hlc.address = ax12_g.p.address;
              hlc.args[0] = ax12_g.p.position;
              hlc.args[1] = ax12_g.p.speed;
              ax12_queue_command(&hlc);
            } while (0);
            break;
          case CAN_AX12_SET_TORQUE_ENABLE:
            do {
              struct ax12_hl_command hlc;
              GET_PACKET(ax12_set_torque_enable, ax12_ste, f);
              hlc.command = AX12_HL_SET_TORQUE_ENABLE;
              hlc.address = ax12_ste.p.address;
              hlc.args[0] = ax12_ste.p.enable;
              ax12_queue_command(&hlc);
            } while (0);
            break;
          default:
            break;
          }
        }
        // General messages
        switch (f.eid) {
        case CAN_MSG_SIMULATION_MODE:
          do {
            GET_PACKET(simulation_mode, mode_req, f);
            sim_mode = mode_req.p.mode;
          } while (0);
          break;
        }
    }

}
