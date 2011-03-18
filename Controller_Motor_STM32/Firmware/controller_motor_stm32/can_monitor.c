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

typedef union {
  encoder_msg_t data;
  uint32_t data32[2];
} encoder_can_msg_t;

typedef struct {
  motor_msg_t data;
  uint32_t data32[2];
} motor_can_msg_t;

// Process for communication
static void NORETURN canMonitor_process(void);

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
  proc_new(canMonitor_process, NULL, KERN_MINSTACKSIZE * 8, NULL);
}

static void NORETURN canMonitor_process(void) {
  encoder_can_msg_t msg_enc;
  motor_can_msg_t msg_mot;
  can_tx_frame txm;
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
    msg_enc.data.encoder1_pos = getEncoderPosition(ENCODER3);
    msg_enc.data.encoder2_pos = getEncoderPosition(ENCODER4);
    msg_enc.data.encoder1_dir = getEncoderDirection(ENCODER3);
    msg_enc.data.encoder2_dir = getEncoderDirection(ENCODER4);

    txm.data32[0] = msg_enc.data32[0];
    txm.data32[1] = msg_enc.data32[1];
    txm.eid = 100;
    can_transmit(CAND1, &txm, ms_to_ticks(10));

    // Sending MOTOR3 data
    msg_mot.data.position = mc_getPosition(MOTOR3);
    msg_mot.data.speed = mc_getSpeed(MOTOR3);

    txm.data32[0] = msg_mot.data32[0];
    txm.data32[1] = msg_mot.data32[1];
    txm.eid = 101;
    can_transmit(CAND1, &txm, ms_to_ticks(10));

    // Sending MOTOR4 data
    msg_mot.data.position = mc_getPosition(MOTOR4);
    msg_mot.data.speed = mc_getSpeed(MOTOR4);

    txm.data32[0] = (int32_t)mc_getPosition(MOTOR4);//msg_mot.data32[0];
    txm.data32[1] = (int32_t)mc_getSpeed(MOTOR4);//msg_mot.data32[1];
    txm.eid = 102;
    can_transmit(CAND1, &txm, ms_to_ticks(10));

    // Wait for the next transmission timer
    timer_waitEvent(&timer_can);
  }
}

