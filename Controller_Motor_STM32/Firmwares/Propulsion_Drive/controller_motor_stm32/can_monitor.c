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

#define CONTROL_ODOMETRY 0

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
  //encoder_can_msg_t msg_enc;
  motor_can_msg_t msg_mot;
  odometry_can_msg_t msg_odo;
  ghost_can_msg_t msg_ghost;
  status_can_msg_t status_msg;
  can_tx_frame txm;
  robot_state_t odometry;
  float u;
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

    // Sending ENCODER3 and ENCODER4 data
    /*msg_enc.data.encoder1_pos = getEncoderPosition(ENCODER3);
    msg_enc.data.encoder2_pos = getEncoderPosition(ENCODER4);
    msg_enc.data.encoder1_dir = getEncoderDirection(ENCODER3);
    msg_enc.data.encoder2_dir = getEncoderDirection(ENCODER4);

    txm.data32[0] = msg_enc.data32[0];
    txm.data32[1] = msg_enc.data32[1];
    txm.eid = CAN_MSG_ENCODERS34;
    can_transmit(CAND1, &txm, ms_to_ticks(10));*/

    // Sending odometry data if not in HIL mode or motor commands if in HIL mode
    if (mode != ROBOT_MODE_HIL) {
      odo_getState(CONTROL_ODOMETRY, &odometry);
      msg_odo.data.x = (int16_t)(odometry.x * 1000.0);
      msg_odo.data.y = (int16_t)(odometry.y * 1000.0);
      odometry.theta = fmodf(odometry.theta, 2*M_PI);
      if (odometry.theta > M_PI)
        odometry.theta -= 2*M_PI;
      if (odometry.theta < -M_PI)
        odometry.theta += 2*M_PI;
      msg_odo.data.theta = (int16_t)(odometry.theta * 10000.0);

      txm.data32[0] = msg_odo.data32[0];
      txm.data32[1] = msg_odo.data32[1];
      txm.eid = CAN_MSG_ODOMETRY;
      can_transmit(CAND1, &txm, ms_to_ticks(10));
    } else {
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
    }

    // Wait before sending the other packets
    timer_waitEvent(&timer_can);
    timer_add(&timer_can);

    // Sending ghost state
    msg_ghost.data.state = dd_get_ghost_state(&odometry, &u);
    msg_ghost.data.x = (int16_t)(odometry.x * 1000.0);
    msg_ghost.data.y = (int16_t)(odometry.y * 1000.0);
    msg_ghost.data.theta = (int16_t)(odometry.theta * 10000.0);
    msg_ghost.data.u = (uint8_t)(u * 255);
    txm.data32[0] = msg_ghost.data32[0];
    txm.data32[1] = msg_ghost.data32[1];
    txm.eid = CAN_MSG_GHOST;
    can_transmit(CAND1, &txm, ms_to_ticks(10));

    status_msg.data.is_moving = tc_is_working(MOTOR1 | MOTOR2 | MOTOR3 | MOTOR4);
    txm.data32[0] = status_msg.data32[0];
    txm.data32[1] = status_msg.data32[1];
    txm.eid = CAN_MSG_STATUS;
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

    move_can_msg_t move_msg;
    turn_can_msg_t turn_msg;
    odometry_can_msg_t odometry_msg;
    stop_can_msg_t stop_msg;
    controller_mode_can_msg_t controller_mode_msg;
    bezier_can_msg_t bezier_msg;
    bezier_limits_can_msg_t bezier_limits_msg;

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
          case CAN_MSG_MOVE:
            move_msg.data32[0] = frame.data32[0];
            move_msg.data32[1] = frame.data32[1];
            if (!tc_is_working(TC_MASK(DD_LINEAR_SPEED_TC) | TC_MASK(DD_ROTATIONAL_SPEED_TC)))
              dd_move(move_msg.data.distance / 1000.0, move_msg.data.speed / 1000.0, move_msg.data.acceleration / 1000.0);
            break;
          case CAN_MSG_TURN:
            turn_msg.data32[0] = frame.data32[0];
            turn_msg.data32[1] = frame.data32[1];
            if (!tc_is_working(TC_MASK(DD_LINEAR_SPEED_TC) | TC_MASK(DD_ROTATIONAL_SPEED_TC)))
              dd_turn(turn_msg.data.angle / 10000.0, turn_msg.data.speed / 1000.0, turn_msg.data.acceleration / 1000.0);
            break;
          case CAN_MSG_BEZIER_ADD:
            bezier_msg.data32[0] = frame.data32[0];
            bezier_msg.data32[1] = frame.data32[1];
            dd_add_bezier(bezier_msg.data.x_end/1000.0, bezier_msg.data.y_end/1000.0,
                          bezier_msg.data.d1/100.0, bezier_msg.data.d2/100.0,
                          bezier_msg.data.theta_end/100.0, bezier_msg.data.v_end/1000.0);
            break;
          case CAN_MSG_BEZIER_LIMITS:
            bezier_limits_msg.data32[0] = frame.data32[0];
            bezier_limits_msg.data32[1] = frame.data32[1];
            dd_adjust_limits(bezier_limits_msg.data.v_max/1000.0,
                             bezier_limits_msg.data.at_max/1000.0,
                             bezier_limits_msg.data.ar_max/1000.0);
            break;
          case CAN_MSG_STOP:
            stop_msg.data32[0] = frame.data32[0];
            stop_msg.data32[1] = frame.data32[1];
            dd_interrupt_trajectory(stop_msg.data.rot_acc, stop_msg.data.lin_acc);
            break;
          case CAN_MSG_ODOMETRY_SET:
            odometry_msg.data32[0] = frame.data32[0];
            odometry_msg.data32[1] = frame.data32[1];
            odometry.x = ((float)odometry_msg.data.x) / 1000.0;
            odometry.y = ((float)odometry_msg.data.y) / 1000.0;
            odometry.theta = ((float)odometry_msg.data.theta) / 10000.0;
            odo_setState(CONTROL_ODOMETRY, &odometry);
            break;
          case CAN_MSG_ODOMETRY:
            // We should only receive such message in HIL mode
            if (mode == ROBOT_MODE_HIL) {
              odometry_msg.data32[0] = frame.data32[0];
              odometry_msg.data32[1] = frame.data32[1];
              odometry.x = ((float)odometry_msg.data.x) / 1000.0;
              odometry.y = ((float)odometry_msg.data.y) / 1000.0;
              odometry.theta = ((float)odometry_msg.data.theta) / 10000.0;
              odo_setState(CONTROL_ODOMETRY, &odometry);
            }
            break;
          case CAN_MSG_CONTROLLER_MODE:
            controller_mode_msg.data32[0] = frame.data32[0];
            controller_mode_msg.data32[1] = frame.data32[0];
            if (controller_mode_msg.data.mode == 1) {
              mc_change_mode(MOTOR3, CONTROLLER_MODE_HIL);
              mc_change_mode(MOTOR4, CONTROLLER_MODE_HIL);
              odo_disable(CONTROL_ODOMETRY);
              mode = ROBOT_MODE_HIL;
            } else {
              mc_change_mode(MOTOR3, CONTROLLER_MODE_NORMAL);
              mc_change_mode(MOTOR4, CONTROLLER_MODE_NORMAL);
              odo_restart(CONTROL_ODOMETRY);
              mode = ROBOT_MODE_NORMAL;
            }
            break;
          }
        }
      }
    }
}
