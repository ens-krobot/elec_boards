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

#define ROBOT_MODE_NORMAL  0
#define ROBOT_MODE_HIL     1

// Processes stacks
PROC_DEFINE_STACK(stack_can_send, KERN_MINSTACKSIZE * 8);
PROC_DEFINE_STACK(stack_can_receive, KERN_MINSTACKSIZE * 8);

// globals
volatile uint8_t mode;

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

  mode = ROBOT_MODE_NORMAL;

  // Start communication process
  proc_new(canMonitor_process, NULL, sizeof(stack_can_send), stack_can_send);
  proc_new(canMonitorListen_process, NULL, sizeof(stack_can_receive), stack_can_receive);
}

static void NORETURN canMonitor_process(void) {
  encoder_can_msg_t msg_enc;
  pump_state_can_msg_t msg_pumps;
  status_can_msg_t msg_status;
  lift_position_can_msg_t msg_lift_position;
  can_tx_frame txm;
  Timer timer_can;

  // Initialize constant parameters of TX frame
  txm.dlc = 8;
  txm.rtr = 0;
  txm.ide = 1;
  txm.sid = 0;

  timer_setDelay(&timer_can, ms_to_ticks(5));
  timer_setEvent(&timer_can);
  while(1) {

    timer_add(&timer_can);

    // Sending ENCODER2 and ENCODER4 data
    msg_enc.data.encoder1_pos = getEncoderPosition(ENCODER2);
    msg_enc.data.encoder2_pos = getEncoderPosition(ENCODER4);
    msg_enc.data.encoder1_dir = getEncoderDirection(ENCODER2);
    msg_enc.data.encoder2_dir = getEncoderDirection(ENCODER4);

    txm.data32[0] = msg_enc.data32[0];
    txm.data32[1] = msg_enc.data32[1];
    txm.eid = CAN_MSG_LIFT_ENCODERS;
    can_transmit(CAND1, &txm, ms_to_ticks(10));

    // Sending TC status
    msg_status.data.is_moving = tc_is_working(MOTOR1 | MOTOR2 | MOTOR3 | MOTOR4);
    txm.data32[0] = msg_status.data32[0];
    txm.data32[1] = msg_status.data32[1];
    txm.eid = CAN_MSG_EFFECTOR_STATUS;
    can_transmit(CAND1, &txm, ms_to_ticks(10));

    // Sending pump states
    msg_pumps.data.left_pump = motorGetSpeed(MOTOR1);
    msg_pumps.data.right_pump = motorGetSpeed(MOTOR2);

    txm.data32[0] = msg_pumps.data32[0];
    txm.data32[1] = msg_pumps.data32[1];
    txm.eid = CAN_MSG_PUMP_STATE;
    can_transmit(CAND1, &txm, ms_to_ticks(10));

    // Wait for the next transmission timer
    timer_waitEvent(&timer_can);
    timer_add(&timer_can);

    // Sending lifts awaited positions
    msg_lift_position.data.left_position = lc_get_position(LC_LEFT_LIFT);
    msg_lift_position.data.right_position = lc_get_position(LC_RIGHT_LIFT);
    txm.data32[0] = msg_lift_position.data32[0];
    txm.data32[1] = msg_lift_position.data32[1];
    txm.eid = CAN_MSG_LIFT_POSITION;
    can_transmit(CAND1, &txm, ms_to_ticks(10));

    // Sending TC status
    msg_status.data.is_moving =
      is_homing_finished(LC_LEFT_LIFT) << 1 | is_homing_finished(LC_RIGHT_LIFT);
    txm.data32[0] = msg_status.data32[0];
    txm.data32[1] = msg_status.data32[1];
    txm.eid = CAN_MSG_HOMING_STATUS;
    can_transmit(CAND1, &txm, ms_to_ticks(10));

    // Wait for the next transmission timer
    timer_waitEvent(&timer_can);
  }
}

static void NORETURN canMonitorListen_process(void) {

    can_rx_frame frame;
    bool received = false;
    can_tx_frame txm;
    uint8_t end_stops;

    //controller_mode_can_msg_t controller_mode_msg;

    switch_status end_courses_msg;
    lift_cmd_can_msg_t lift_cmd_msg;
    pump_cmd_can_msg_t pump_cmd_msg;
    homing_cmd_can_msg_t homing_cmd_msg;

    // Initialize constant parameters of TX frame
    txm.dlc = 8;
    txm.rtr = 0;
    txm.ide = 1;
    txm.sid = 0;

    while (1) {
      received = can_receive(CAND1, &frame, ms_to_ticks(100));
      if (received) {
        if (frame.rtr == 1) {
          // Handle requests
          switch (frame.eid) {
          default:
            break;
          }
        } else {
          // Handle commands and other informations
          switch (frame.eid) {
        /*case CAN_MSG_CONTROLLER_MODE:
            controller_mode_msg.data32[0] = frame.data32[0];
            controller_mode_msg.data32[1] = frame.data32[0];
            if (controller_mode_msg.data.mode == 1) {
              mc_change_mode(MOTOR3, CONTROLLER_MODE_HIL);
              mc_change_mode(MOTOR4, CONTROLLER_MODE_HIL);
              odo_disable();
              mode = ROBOT_MODE_HIL;
            } else {
              mc_change_mode(MOTOR3, CONTROLLER_MODE_NORMAL);
              mc_change_mode(MOTOR4, CONTROLLER_MODE_NORMAL);
              odo_restart();
              mode = ROBOT_MODE_NORMAL;
            }
            break;*/
          case CAN_SWITCH_STATUS_1:
            end_courses_msg.d[0] = frame.data32[0];
            end_courses_msg.d[1] = frame.data32[1];
            end_stops = 0;
            if (!end_courses_msg.p.sw4)
              end_stops |= LC_LEFT_BOTTOM;
            if (!end_courses_msg.p.sw5)
              end_stops |= LC_RIGHT_BOTTOM;
            lc_end_stop_reached(end_stops);
            //if (end_courses_msg.p.sw1) {
            //  lc_release();
            //  motorSetSpeed(MOTOR2,0);
            break;
          case CAN_MSG_LIFT_CMD:
            lift_cmd_msg.data32[0] = frame.data32[0];
            lift_cmd_msg.data32[1] = frame.data32[1];
            if (lift_cmd_msg.data.left_lift >= 0)
              lc_goto_position(LC_LEFT_LIFT, lift_cmd_msg.data.left_lift);
            if (lift_cmd_msg.data.right_lift >= 0)
              lc_goto_position(LC_RIGHT_LIFT, lift_cmd_msg.data.right_lift);
            break;
          case CAN_MSG_PUMP_CMD:
            pump_cmd_msg.data32[0] = frame.data32[0];
            pump_cmd_msg.data32[1] = frame.data32[1];
            if (pump_cmd_msg.data.left_pump >= 0)
              motorSetSpeed(MOTOR1, pump_cmd_msg.data.left_pump);
            if (pump_cmd_msg.data.right_pump >= 0)
              motorSetSpeed(MOTOR2, pump_cmd_msg.data.right_pump);
            break;
          case CAN_MSG_HOMING_CMD:
            homing_cmd_msg.data32[0] = frame.data32[0];
            homing_cmd_msg.data32[1] = frame.data32[1];
            if (homing_cmd_msg.data.left_lift > 0)
              lc_homing(LC_LEFT_LIFT, homing_cmd_msg.data.left_lift);
            if (homing_cmd_msg.data.right_lift > 0)
              lc_homing(LC_RIGHT_LIFT, homing_cmd_msg.data.right_lift);
          }
        }
      }
    }
}
