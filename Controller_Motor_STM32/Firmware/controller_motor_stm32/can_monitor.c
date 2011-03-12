/*
 * can_monitor.c
 * -------------
 * Copyright : (c) 2011, Xavier Lagorce <Xavier.Lagorce@crans.org>
 * Licence   : BSD3
 *
 * This file is a part of [kro]bot.
 */

#include "can_monitor.h"

Timer timer_can;
can_tx_frame txm;

typedef struct {
  uint16_t encoder1_pos  __attribute__((__packed__));
  uint16_t encoder2_pos  __attribute__((__packed__));
  uint8_t encoder1_dir;
  uint8_t encoder2_dir;
  uint16_t padding  __attribute__((__packed__));
} encoder_msg_t;

typedef union {
  encoder_msg_t data_f;
  uint32_t data32[2];
} encoder_can_msg_t;

// Callback for communication
static void canMonitorCallback(void* data);

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

  // Initialize constant parameters of TX frame
  txm.dlc = 8;
  txm.rtr = 0;
  txm.ide = 1;
  txm.sid = 0;
  txm.eid = 0;


  // Start communication thread
  timer_setSoftint(&timer_can, canMonitorCallback, NULL);
  timer_setDelay(&timer_can, ms_to_ticks(10));
  timer_add(&timer_can);
}

static void canMonitorCallback(void* data) {
  (void)data;

  encoder_can_msg_t msg;

  // Sending ENCODER1 and ENCODER2 data
  msg.data_f.encoder1_pos = getEncoderPosition(ENCODER1);
  msg.data_f.encoder2_pos = getEncoderPosition(ENCODER2);
  msg.data_f.encoder1_dir = getEncoderDirection(ENCODER1);
  msg.data_f.encoder2_dir = getEncoderDirection(ENCODER2);

  txm.data32[0] = msg.data32[0];
  txm.data32[1] = msg.data32[1];
  txm.eid = 100;
  can_transmit(CAND1, &txm, ms_to_ticks(1));

  // Sending ENCODER3 and ENCODER4 data
  msg.data_f.encoder1_pos = getEncoderPosition(ENCODER3);
  msg.data_f.encoder2_pos = getEncoderPosition(ENCODER4);
  msg.data_f.encoder1_dir = getEncoderDirection(ENCODER3);
  msg.data_f.encoder2_dir = getEncoderDirection(ENCODER4);

  txm.data32[0] = msg.data32[0];
  txm.data32[1] = msg.data32[1];
  txm.eid = 101;
  can_transmit(CAND1, &txm, ms_to_ticks(1));

  timer_add(&timer_can);
}
