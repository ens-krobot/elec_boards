/*
 * can_monitor.c
 * -------------
 * Copyright : (c) 2011, Xavier Lagorce <Xavier.Lagorce@crans.org>
 * Licence   : BSD3
 *
 * This file is a part of [kro]bot.
 */

#include "can_monitor.h"
#include "hw/hw_led.h"

PROC_DEFINE_STACK(stack_can_send, KERN_MINSTACKSIZE * 8);
PROC_DEFINE_STACK(stack_can_receive, KERN_MINSTACKSIZE * 8);

// Structures to represent data into CAN messages
typedef struct {
  uint16_t encoder1_pos  __attribute__((__packed__));
  uint16_t encoder2_pos  __attribute__((__packed__));
  uint8_t encoder1_dir;
  uint8_t encoder2_dir;
  uint16_t padding  __attribute__((__packed__));
} encoder_msg_t;

typedef struct {
  float position __attribute__((__packed__));
  float speed    __attribute__((__packed__));
} motor_msg_t;

typedef struct {
  uint8_t is_moving; // 1 if a movement action is in progress, 0 if it's finished
} status_msg_t;

typedef struct {
  int16_t x __attribute__((__packed__));     // X position in mm (fixed point representation...)
  int16_t y __attribute__((__packed__));     // Y position in mm
  int16_t theta __attribute__((__packed__)); // angle in 1/100 degrees
} odometry_msg_t;

typedef struct {
  int32_t distance      __attribute__((__packed__));  // Distance in mm (fixed point representation...)
  uint16_t speed        __attribute__((__packed__));  // Speed in mm/s
  uint16_t acceleration __attribute__((__packed__));  // Acceleration in mm/s^2
} move_msg_t;

typedef struct {
  int32_t angle         __attribute__((__packed__));  // angle in 1/100 degrees (fixed point representation...)
  uint16_t speed        __attribute__((__packed__));  // Speed in 1/100 degrees/s
  uint16_t acceleration __attribute__((__packed__));  // Acceleration in 1/100 degrees/s^2
} turn_msg_t;

typedef struct {
  uint8_t stop;  // stop everything
} stop_msg_t;

// Union to manipulate CAN messages' data easily
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
  move_msg_t data;
  uint32_t data32[2];
} move_can_msg_t;

typedef union {
  turn_msg_t data;
  uint32_t data32[2];
} turn_can_msg_t;

typedef union {
  stop_msg_t data;
  uint32_t data32[2];
} stop_can_msg_t;

// Can messages IDs
#define CAN_MSG_ENCODERS34 100 // encoder_can_msg_t
#define CAN_MSG_MOTOR3 101 // motor_can_msg_t
#define CAN_MSG_MOTOR4 102 // motor_can_msg_t
#define CAN_MSG_STATUS 103 // status_can_msg_t
#define CAN_MSG_ODOMETRY 104 // odometry_can_msg_t
#define CAN_MSG_MOVE 201 // move_can_msg_t
#define CAN_MSG_TURN 202 // turn_can_msg_t
#define CAN_MSG_ODOMETRY_SET 203 // odometry_can_msg_t
#define CAN_MSG_STOP 204 // stop_can_msg_t

// Process for communication
static void NORETURN canMonitor_process(void);
static void NORETURN canMonitorListen_process(void);

void canMonitorInit(void) {
  can_config cfg;

  // Configure CAN driver
  cfg.mcr = CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP;
  cfg.btr = CAN_BTR_SJW(0) | CAN_BTR_TS1(8) | CAN_BTR_TS2(1) | CAN_BTR_BRP(6);
  cfg.n_filters = 0;
  cfg.filters = NULL;

  // Initialize CAN driver
  can_init();
  can_start(CAND1, &cfg);

  // Start communication process
  proc_new(canMonitor_process, NULL, sizeof(stack_can_send), stack_can_send);
  proc_new(canMonitorListen_process, NULL, sizeof(stack_can_receive), stack_can_receive);
}

static void NORETURN canMonitor_process(void) {
  //encoder_can_msg_t msg_enc;
  motor_can_msg_t msg_mot;
  odometry_can_msg_t msg_odo;
  can_tx_frame txm;
  robot_state_t odometry;
  Timer timer_can;

  // Initialize constant parameters of TX frame
  txm.dlc = 8;
  txm.rtr = 0;
  txm.ide = 1;
  txm.sid = 0;

  timer_setDelay(&timer_can, ms_to_ticks(10));
  timer_setEvent(&timer_can);
  while(1) {

    timer_add(&timer_can);

    // Sending ENCODER3 and ENCODER4 data
    /*msg_enc.data.encoder1_pos = getEncoderPosition(ENCODER3);
    msg_enc.data.encoder2_pos = getEncoderPosition(ENCODER4);
    msg_enc.data.encoder1_dir = getEncoderDirection(ENCODER3);
    msg_enc.data.encoder2_dir = getEncoderDirection(ENCODER4);

    txm.data32[0] = msg_enc.data32[0];
    txm.data32[1] = msg_enc.data32[1];
    txm.eid = CAN_MSG_ENCODERS34;
    can_transmit(CAND1, &txm, ms_to_ticks(10));*/

    // Sending MOTOR3 data
    msg_mot.data.position = mc_getPosition(MOTOR3);
    msg_mot.data.speed = mc_getSpeed(MOTOR3);

    txm.data32[0] = msg_mot.data32[0];
    txm.data32[1] = msg_mot.data32[1];
    txm.eid = CAN_MSG_MOTOR3;
    can_transmit(CAND1, &txm, ms_to_ticks(10));

    // Sending MOTOR4 data
    msg_mot.data.position = mc_getPosition(MOTOR4);
    msg_mot.data.speed = mc_getSpeed(MOTOR4);

    txm.data32[0] = msg_mot.data32[0];
    txm.data32[1] = msg_mot.data32[1];
    txm.eid = CAN_MSG_MOTOR4;
    can_transmit(CAND1, &txm, ms_to_ticks(10));

    // Sending odometry data
    odo_getState(&odometry);
    msg_odo.data.x = (int16_t)(odometry.x * 1000.0);
    msg_odo.data.y = (int16_t)(odometry.y * 1000.0);
    odometry.theta = fmodf(odometry.theta, 360.0);
    if (odometry.theta > 180.0)
      odometry.theta -= 360.0;
    if (odometry.theta < -180.0)
      odometry.theta += 360.0;
    msg_odo.data.theta = (int16_t)(odometry.theta * 100.0);

    txm.data32[0] = msg_odo.data32[0];
    txm.data32[1] = msg_odo.data32[1];
    txm.eid = CAN_MSG_ODOMETRY;
    can_transmit(CAND1, &txm, ms_to_ticks(10));

    // Wait for the next transmission timer
    timer_waitEvent(&timer_can);
  }
}

static void NORETURN canMonitorListen_process(void) {

    can_rx_frame frame;
    bool received = false;
    can_tx_frame txm;
    robot_state_t odometry;

    status_can_msg_t status_msg;
    move_can_msg_t move_msg;
    turn_can_msg_t turn_msg;
    odometry_can_msg_t odometry_msg;
    stop_can_msg_t stop_msg;

    tc_robot_t robot;

    // Initialize constant parameters of TX frame
    txm.dlc = 8;
    txm.rtr = 0;
    txm.ide = 1;
    txm.sid = 0;

    // Initialize robot representation
    robot.left_wheel = MOTOR3;
    robot.right_wheel = MOTOR4;
    robot.wheel_radius = 0.049245;
    robot.shaft_width = 0.259;

    while (1) {
      received = can_receive(CAND1, &frame, ms_to_ticks(100));
      if (received) {
        if (frame.rtr == 1) {
          // Handle requests
          switch (frame.eid) {
          case CAN_MSG_STATUS:
            status_msg.data.is_moving = tc_is_working(MOTOR1 | MOTOR2 | MOTOR3 | MOTOR4);
            txm.data32[0] = status_msg.data32[0];
            txm.data32[1] = status_msg.data32[1];
            txm.eid = CAN_MSG_STATUS;
            can_transmit(CAND1, &txm, ms_to_ticks(10));
            break;
          }
        } else {
          // Handle commands and other informations
          switch (frame.eid) {
          case CAN_MSG_MOVE:
            move_msg.data32[0] = frame.data32[0];
            move_msg.data32[1] = frame.data32[1];
            if (!tc_is_working(MOTOR3 | MOTOR4))
              tc_move(&robot, move_msg.data.distance / 1000.0, move_msg.data.speed / 1000.0, move_msg.data.acceleration / 1000.0);
            break;
          case CAN_MSG_TURN:
            turn_msg.data32[0] = frame.data32[0];
            turn_msg.data32[1] = frame.data32[1];
            if (!tc_is_working(MOTOR3 | MOTOR4))
              tc_turn(&robot, turn_msg.data.angle / 100.0, turn_msg.data.speed / 100.0, turn_msg.data.acceleration / 100.0);
            break;
          case CAN_MSG_STOP:
            stop_msg.data32[0] = frame.data32[0];
            stop_msg.data32[1] = frame.data32[1];
            if (stop_msg.data.stop == 1) {
              tc_delete_controller(MOTOR3);
              tc_delete_controller(MOTOR4);
            }
            break;
          case CAN_MSG_ODOMETRY_SET:
            odometry_msg.data32[0] = frame.data32[0];
            odometry_msg.data32[1] = frame.data32[1];
            odometry.x = ((float)odometry_msg.data.x) / 1000.0;
            odometry.y = ((float)odometry_msg.data.y) / 1000.0;
            odometry.theta = ((float)odometry_msg.data.theta) / 100.0;
            odo_setState(&odometry);
            break;
          }
        }
      }
    }
}
