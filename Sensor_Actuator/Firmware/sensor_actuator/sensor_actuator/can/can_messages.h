/**
 * Sensor-Actuator board messages
 *
 * Header file for the Sensor and Actuator CAN messages
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

#ifndef CAN_MESSAGES_H__
#define CAN_MESSAGES_H__

/**
 * Message IDs
 */

// Beacon

#define CAN_BEACON_POSITION 301          // beacon_position
#define CAN_BEACON_LOWLEVEL_POSITION 302 // beacon_lowlevel_position
#define CAN_BEACON_CALIBRATION 303       // beacon_calibration

// Switches

#define CAN_SWITCH_STATUS_1 311          // switch_status
#define CAN_SWITCH_STATUS_2 312          // switch_status
#define CAN_SWITCH_SET 313               // switch_request


// ADC

#define CAN_ADC_VALUES_1 321             // adc_values
#define CAN_ADC_VALUES_2 322             // adc_values

// Battery monitoring

#define CAN_BATTERY_STATUS_1 331         // battery_status
#define CAN_BATTERY_STATUS_2 332         // battery_status

// AX12

#define CAN_AX12_STATE 341               // ax12_state
#define CAN_AX12_REQUEST_STATE 342       // ax12_request_state
#define CAN_AX12_GOTO 343                // ax12_goto
//#define CAN_AX12_RESET 344             // Not used anymore
#define CAN_AX12_SET_TORQUE_ENABLE 345   // ax12_set_torque_enable

/****************************************************************************/

/**
 * Message Packets
 */

/**
 * Beacon messages
 */

// Position of the opponent
struct beacon_position_pkt {
    uint16_t angle;    // in 1/10000th of radians [0; 2*Pi[
    uint16_t distance; // in mm [0; 65536[
    uint16_t period;   // in 1/10000th of seconds [0; 1[
} __attribute__((packed));



// Beacon low-level position
struct beacon_lowlevel_position_pkt {
    uint16_t angle;  // in 1/10000th of radians [0; 2*Pi[
    uint16_t width;  // in 1/100000th of radians [0; Pi/5[
    uint32_t period; // in timer ticks
} __attribute__((packed));



// Beacon calibration
struct beacon_calibration_pkt {
    uint16_t width;    // in 1/100000th of radians [0; Pi/5[
    uint16_t distance; // in mm [0; 65536[
} __attribute__((packed));


/**
 * Switch messages
 */

// Switch status
struct switch_status_pkt {
    uint8_t sw1;
    uint8_t sw2;
    uint8_t sw3;
    uint8_t sw4;
    uint8_t sw5;
    uint8_t sw6;
    uint8_t sw7;
    uint8_t sw8;
};

// Switch request
struct switch_request_pkt {
    uint8_t num;
    uint8_t state;
};

/**
 * ADC messages
 */
// ADC Values
struct adc_values_pkt {
    uint16_t val1;
    uint16_t val2;
    uint16_t val3;
    uint16_t val4;
} __attribute__((packed));

/**
 * Battery monitoring
 */
struct battery_status_pkt {
    uint16_t elem[4]; // in 1/10000th volts [0; 6.5536[
} __attribute__((packed));


/**
 * AX-12
 */
struct ax12_state_pkt {
    uint8_t address;
    uint16_t position;
    uint16_t speed;
    uint16_t torque;
} __attribute__((packed));

struct ax12_request_state_pkt {
    uint8_t address;
} __attribute__((packed));

struct ax12_goto_pkt {
    uint8_t address;
    uint16_t position;
    uint16_t speed;
} __attribute__((packed));

struct ax12_set_torque_enable_pkt {
    uint8_t address;
    uint8_t enable;
};

/****************************************************************************/

/**
 * Typedefs
 */

typedef union {
    struct beacon_position_pkt p;
    uint32_t d[2];
} beacon_position;

typedef union {
    struct beacon_lowlevel_position_pkt p;
    uint32_t d[2];
} beacon_lowlevel_position;

typedef union {
    struct beacon_calibration_pkt p;
    uint32_t d[2];
} beacon_calibration;

typedef union {
    struct switch_status_pkt p;
    uint32_t d[2];
} switch_status;

typedef union {
    struct switch_request_pkt p;
    uint32_t d[2];
} switch_request;

typedef union {
    struct adc_values_pkt p;
    uint32_t d[2];
} adc_values;

typedef union {
    struct battery_status_pkt p;
    uint32_t d[2];
} battery_status;

typedef union {
    struct ax12_state_pkt p;
    uint32_t d[2];
} ax12_state;

typedef union {
    struct ax12_request_state_pkt p;
    uint32_t d[2];
} ax12_request_state;

typedef union {
    struct ax12_goto_pkt p;
    uint32_t d[2];
} ax12_goto;

typedef union {
    struct ax12_set_torque_enable_pkt p;
    uint32_t d[2];
} ax12_set_torque_enable;

#endif
