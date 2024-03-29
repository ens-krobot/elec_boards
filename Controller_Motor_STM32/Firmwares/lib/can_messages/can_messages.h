/*
 * can_messages.h
 * --------------
 * Copyright : (c) 2011, Xavier Lagorce <Xavier.Lagorce@crans.org>
 * Licence   : BSD3
 *
 * This file is a part of [kro]bot.
 */

#ifndef __CAN_MESSAGES_H
#define __CAN_MESSAGES_H

#include <stdint.h>

/* +-----------------------------------------------------------------+
   | CAN messages IDs                                                |
   +-----------------------------------------------------------------+ */

// Data sent by the card
#define CAN_MSG_ENCODERS12 99 // encoder_can_msg_t
#define CAN_MSG_ENCODERS34 100 // encoder_can_msg_t
#define CAN_MSG_MOTOR3 101 // motor_can_msg_t
#define CAN_MSG_MOTOR4 102 // motor_can_msg_t
#define CAN_MSG_STATUS 103 // status_can_msg_t
#define CAN_MSG_ODOMETRY 104 // odometry_can_msg_t
#define CAN_MSG_GHOST 105 // ghost_can_msg_t
#define CAN_MSG_CONTROL_ERROR 106 // error_can_msg_t
#define CAN_MSG_ODOMETRY_INDEP 107 // odometry_can_msg_t
#define CAN_MSG_MOTOR2 108 // motor_can_msg_t

#define CAN_MSG_LIFT_ENCODERS 131 // encoder_can_msg_t
#define CAN_MSG_PUMP_STATE 132 // pump_state_can_msg_t
#define CAN_MSG_EFFECTOR_STATUS 133 // status_can_msg_t
#define CAN_MSG_LIFT_POSITION 134 // lift_positions_can_msg_t
#define CAN_MSG_HOMING_STATUS 135 // status_can_msg_t

// Received commands
#define CAN_MSG_MOVE 201 // move_can_msg_t
#define CAN_MSG_TURN 202 // turn_can_msg_t
#define CAN_MSG_ODOMETRY_SET 203 // odometry_can_msg_t
#define CAN_MSG_STOP 204 // stop_can_msg_t
#define CAN_MSG_SIMULATION_MODE 205 // simulation_mode_can_msg_t
#define CAN_MSG_BEZIER_ADD 206 // bezier_can_msg_t
#define CAN_MSG_BEZIER_LIMITS 207 // bezier_limits_can_msg_t
#define CAN_MSG_MOTOR_COMMAND 208 // motor_command_can_msg_t
#define CAN_MSG_ODOMETRY_INDEP_SET 209 // odometry_can_msg_t
#define CAN_MSG_CONTROLLER_ACTIVATION 210 // controller_activation_can_msg_t
#define CAN_MSG_DRIVE_ACTIVATION 211 // controller_activation_can_msg_t
#define CAN_MSG_TORQUE_LIMIT 212 // torque_limit_can_msg_t
#define CAN_MSG_DRIVE_TORQUE_LIMIT 213 // drive_torque_limit_can_msg_t
#define CAN_MSG_MOVE_X 215 // move_can_msg_t
#define CAN_MSG_MOVE_Y 216 // move_can_msg_t
#define CAN_MSG_OMNI_LIMITS 217 // omni_limits_can_msg_t
#define CAN_MSG_OMNI_GOTO 218 // omni_goto_can_msg_t
#define CAN_MSG_LOCK_TARGET 219 // lock_target_can_msg_t
#define CAN_MSG_UNLOCK_TARGET 220
#define CAN_MSG_LOCK_TARGET_STATUS 221 // lock_target_status_can_msg_t

#define CAN_MSG_LIFT_CMD 231 // lift_cmd_msg_t
#define CAN_MSG_PUMP_CMD 232 // pump_cmd_msg_t
#define CAN_MSG_HOMING_CMD 233 // homing_command_msg_t

/* +-----------------------------------------------------------------+
   | Constants for messages                                          |
   +-----------------------------------------------------------------+ */
#define SIMULATION_MODE_NO 0
#define SIMULATION_MODE_NORMAL 1
#define SIMULATION_MODE_HIL 2

/* +-----------------------------------------------------------------+
   | CAN messages data structures                                    |
   +-----------------------------------------------------------------+ */

// Information about two encoders
typedef struct {
  uint16_t encoder1_pos  __attribute__((__packed__));
  uint16_t encoder2_pos  __attribute__((__packed__));
  uint8_t encoder1_dir;
  uint8_t encoder2_dir;
  uint16_t padding  __attribute__((__packed__));
} encoder_msg_t;

// Position and speed of one controller motor
typedef struct {
  float position __attribute__((__packed__)); // angle in radians
  float speed    __attribute__((__packed__)); // speed in rad/s (estimation)
} motor_msg_t;

// Trajectory controller states
typedef struct {
  uint8_t is_moving;
} status_msg_t;

// Error packet
typedef struct {
  uint8_t err1;
  uint8_t err2;
} error_msg_t;

// Robot state
typedef struct {
  int16_t x __attribute__((__packed__));     // X position in mm (fixed point representation...)
  int16_t y __attribute__((__packed__));     // Y position in mm
  int16_t theta __attribute__((__packed__)); // angle in 1/10000 radians
} odometry_msg_t;

// Ghost state for differential drive system
typedef struct {
  int16_t x __attribute__((__packed__));     // X position in mm (fixed point representation...)
  int16_t y __attribute__((__packed__));     // Y position in mm
  int16_t theta __attribute__((__packed__)); // angle in 1/10000 radians
  uint8_t u; // Parameter on the spline between 0 and 255
  uint8_t state; // 1 if trajectory in progress, 0 else
} ghost_msg_t;

// Pumps voltage status
typedef struct {
  int16_t left_pump __attribute__((__packed__));
  int16_t right_pump  __attribute__((__packed__));
} pump_state_msg_t;

// Torque limitation on some motors
typedef struct {
  uint8_t motor         __attribute__((__packed__)); // Logic OR of bits corresponding to motors
  uint16_t limit        __attribute__((__packed__)); // Torque limit (between 0 and 3600)
} torque_limit_msg_t;

// Torque limitation on propulsion drive
typedef struct {
  uint16_t limit        __attribute__((__packed__)); // Torque limit (between 0 and 3600)
} drive_torque_limit_msg_t;

// Move command
typedef struct {
  int32_t distance      __attribute__((__packed__));  // Distance in mm (fixed point representation...)
  uint16_t speed        __attribute__((__packed__));  // Speed in mm/s
  uint16_t acceleration __attribute__((__packed__));  // Acceleration in mm/s^2
} move_msg_t;

// Turn command
typedef struct {
  int32_t angle         __attribute__((__packed__));  // angle in 1/10000 radians (fixed point representation...)
  uint16_t speed        __attribute__((__packed__));  // Speed in 1/1000 rad/s
  uint16_t acceleration __attribute__((__packed__));  // Acceleration in 1/1000 radians/s^2
} turn_msg_t;

// Controller activation command
typedef struct {
  uint8_t motor         __attribute__((__packed__)); // Motor ID of the motor to act on
  uint8_t activate      __attribute__((__packed__)); // Activate the controller if non zero
                                                     // else release the control.
} controller_activation_msg_t;

// Propulsion Drive activation command
typedef struct {
  uint8_t activate      __attribute__((__packed__)); // Activate the propulsion drive if non zero
                                                     // else release the control.
} drive_activation_msg_t;

// Add a new Bezier Spline to the wait queue
typedef struct {
  uint16_t x_end:12     __attribute__((__packed__)); // end point x coordinate in mm
  uint16_t y_end:12     __attribute__((__packed__)); // end point y coordinate in mm
  int16_t d1:9          __attribute__((__packed__)); // first branch length in cm
  uint8_t d2:8          __attribute__((__packed__)); // last branch length in cm
  int16_t theta_end:12  __attribute__((__packed__)); // end angle in 1/100 radians
  uint16_t v_end:11     __attribute__((__packed__)); // final speed in mm/s
} bezier_msg_t;

// Modify Bezier Spline trajectory generation limits
typedef struct {
  uint16_t v_max     __attribute__((__packed__)); // max linear speed in mm/s
  uint16_t omega_max __attribute__((__packed__)); // max rotational speed in mrad/s
  uint16_t at_max    __attribute__((__packed__)); // max linear acceleration in mm/s/s
  uint16_t ar_max    __attribute__((__packed__)); // max radial acceleration in mm/s/s
} bezier_limits_msg_t;

// Stop Bezier Spline following and brakes
typedef struct {
  float lin_acc  __attribute__((__packed__)); // Linear acceleration for braking
  float rot_acc  __attribute__((__packed__)); // Rotational acceleration for braking
} stop_msg_t;

// Send of Goto command to the holonomic drive
typedef struct {
  uint16_t x_end:12     __attribute__((__packed__)); // end point x coordinate in mm
  uint16_t y_end:12     __attribute__((__packed__)); // end point y coordinate in mm
  int16_t theta_end:12  __attribute__((__packed__)); // end angle in 1/100 radians
  uint32_t padding:28   __attribute__((__packed__)); // Reserved for futur use
} omni_goto_msg_t;

// Modify holonomic drive dynamical limits
typedef struct {
  uint16_t v_lin_max __attribute__((__packed__)); // max linear speed in mm/s
  uint16_t v_rot_max __attribute__((__packed__)); // max rotational speed in mrad/s
  uint16_t a_lin_max __attribute__((__packed__)); // max linear acceleration in mm/s/s
  uint16_t a_rot_max __attribute__((__packed__)); // max radial acceleration in rad/s/s
} omni_limits_msg_t;

// Target position message
typedef struct {
  int16_t x __attribute__((__packed__));     // X position in mm (fixed point representation...)
  int16_t y __attribute__((__packed__));     // Y position in mm
  int16_t theta __attribute__((__packed__)); // angle in 1/10000 radians
} lock_target_msg_t;

// Target lock status message
typedef struct {
  float lock_error __attribute__((__packed__)); // angular error in rad
  uint8_t lock_status __attribute__((__packed__)); // non-zero if target is currently locked
} lock_target_status_msg_t;

// Select robot mode (normal, simulation or HIL)
typedef struct {
  uint8_t mode;
} simulation_mode_msg_t;

// Command the speed of a particular motor
typedef struct {
  uint8_t motor_id  __attribute__((__packed__));
  int32_t speed     __attribute__((__packed__));
} motor_command_msg_t;

// Lifts command position, a value is ignored if < 0
typedef struct {
  float left_lift __attribute__((__packed__));
  float right_lift  __attribute__((__packed__));
} lift_cmd_msg_t;

// Pumps voltage command, a value is ignored if < 0
typedef struct {
  int16_t left_pump __attribute__((__packed__));
  int16_t right_pump  __attribute__((__packed__));
} pump_cmd_msg_t;

// Pumps voltage command, a value is ignored if < 0
typedef struct {
  float left_lift __attribute__((__packed__));
  float right_lift  __attribute__((__packed__));
} homing_cmd_msg_t;

// Awaited positions of the two lifts
typedef struct {
  float left_position __attribute__((__packed__)); // position in m
  float right_position __attribute__((__packed__)); // position in m
} lift_position_msg_t;

/* +-----------------------------------------------------------------+
   | CAN messages unions for data representation                     |
   +-----------------------------------------------------------------+ */

typedef union {
  encoder_msg_t data;
  uint32_t data32[2];
} encoder_can_msg_t;

typedef union {
  motor_msg_t data;
  uint32_t data32[2];
} motor_can_msg_t;

typedef union {
  status_msg_t data;
  uint32_t data32[2];
} status_can_msg_t;

typedef union {
  error_msg_t data;
  uint32_t data32[2];
} error_can_msg_t;

typedef union {
  odometry_msg_t data;
  uint32_t data32[2];
} odometry_can_msg_t;

typedef union {
  ghost_msg_t data;
  uint32_t data32[2];
} ghost_can_msg_t;

typedef union {
  pump_state_msg_t data;
  uint32_t data32[2];
} pump_state_can_msg_t;

typedef union {
  torque_limit_msg_t data;
  uint32_t data32[2];
} torque_limit_can_msg_t;

typedef union {
  drive_torque_limit_msg_t data;
  uint32_t data32[2];
} drive_torque_limit_can_msg_t;

typedef union {
  move_msg_t data;
  uint32_t data32[2];
} move_can_msg_t;

typedef union {
  turn_msg_t data;
  uint32_t data32[2];
} turn_can_msg_t;

typedef union {
  controller_activation_msg_t data;
  uint32_t data32[2];
} controller_activation_can_msg_t;

typedef union {
  drive_activation_msg_t data;
  uint32_t data32[2];
} drive_activation_can_msg_t;

typedef union {
  bezier_msg_t data;
  uint32_t data32[2];
} bezier_can_msg_t;

typedef union {
  bezier_limits_msg_t data;
  uint32_t data32[2];
} bezier_limits_can_msg_t;

typedef union {
  omni_goto_msg_t data;
  uint32_t data32[2];
} omni_goto_can_msg_t;

typedef union {
  omni_limits_msg_t data;
  uint32_t data32[2];
} omni_limits_can_msg_t;

typedef union {
  lock_target_msg_t data;
  uint32_t data32[2];
} lock_target_can_msg_t;

typedef union {
  lock_target_status_msg_t data;
  uint32_t data32[2];
} lock_target_status_can_msg_t;

typedef union {
  stop_msg_t data;
  uint32_t data32[2];
} stop_can_msg_t;

typedef union {
  simulation_mode_msg_t data;
  uint32_t data32[2];
} simulation_mode_can_msg_t;

typedef union {
  motor_command_msg_t data;
  uint32_t data32[2];
} motor_command_can_msg_t;

typedef union {
  lift_cmd_msg_t data;
  uint32_t data32[2];
} lift_cmd_can_msg_t;

typedef union {
  pump_cmd_msg_t data;
  uint32_t data32[2];
} pump_cmd_can_msg_t;

typedef union {
  homing_cmd_msg_t data;
  uint32_t data32[2];
} homing_cmd_can_msg_t;

typedef union {
  lift_position_msg_t data;
  uint32_t data32[2];
} lift_position_can_msg_t;

#endif /* __CAN_MESSAGES_H */
