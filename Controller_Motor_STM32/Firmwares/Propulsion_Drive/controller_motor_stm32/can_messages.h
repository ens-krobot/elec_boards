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
#define CAN_MSG_ENCODERS34 100 // encoder_can_msg_t
#define CAN_MSG_MOTOR3 101 // motor_can_msg_t
#define CAN_MSG_MOTOR4 102 // motor_can_msg_t
#define CAN_MSG_STATUS 103 // status_can_msg_t
#define CAN_MSG_ODOMETRY 104 // odometry_can_msg_t
#define CAN_MSG_GHOST 105 // ghost_can_msg_t

// Received commands
#define CAN_MSG_MOVE 201 // move_can_msg_t
#define CAN_MSG_TURN 202 // turn_can_msg_t
#define CAN_MSG_ODOMETRY_SET 203 // odometry_can_msg_t
#define CAN_MSG_STOP 204 // stop_can_msg_t
#define CAN_MSG_CONTROLLER_MODE 205 // controller_mode_can_msg_t
#define CAN_MSG_BEZIER_ADD 206 // bezier_can_msg_t
#define CAN_MSG_BEZIER_LIMITS 207 // bezier_limits_can_msg_t
#define CAN_MSG_MOTOR_COMMAND 208 // motor_command_can_msg_t

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
  uint16_t at_max    __attribute__((__packed__)); // max linear acceleration in mm/s/s
  uint16_t ar_max    __attribute__((__packed__)); // max radial acceleration in mm/s/s
} bezier_limits_msg_t;

// Stop Bezier Spline following and brakes
typedef struct {
  float lin_acc  __attribute__((__packed__)); // Linear acceleration for braking
  float rot_acc  __attribute__((__packed__)); // Rotational acceleration for braking
} stop_msg_t;

// Select robot mode (normal or HIL)
typedef struct {
  uint8_t mode;
} controller_mode_msg_t;

// Command the speed of a particular motor
typedef struct {
  uint8_t motor_id;
  int32_t speed;
} motor_command_msg_t;

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
  odometry_msg_t data;
  uint32_t data32[2];
} odometry_can_msg_t;

typedef union {
  ghost_msg_t data;
  uint32_t data32[2];
} ghost_can_msg_t;

typedef union {
  move_msg_t data;
  uint32_t data32[2];
} move_can_msg_t;

typedef union {
  turn_msg_t data;
  uint32_t data32[2];
} turn_can_msg_t;

typedef union {
  bezier_msg_t data;
  uint32_t data32[2];
} bezier_can_msg_t;

typedef union {
  bezier_limits_msg_t data;
  uint32_t data32[2];
} bezier_limits_can_msg_t;

typedef union {
  stop_msg_t data;
  uint32_t data32[2];
} stop_can_msg_t;

typedef union {
  controller_mode_msg_t data;
  uint32_t data32[2];
} controller_mode_can_msg_t;

typedef union {
  motor_command_msg_t data;
  uint32_t data32[2];
} motor_command_can_msg_t;

#endif /* __CAN_MESSAGES_H */
