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

typedef union {
  encoder_msg_t data_f;
  uint32_t data32[2];
} encoder_can_msg_t;

// Process for communication
static void NORETURN canMonitor_process(void);

void canMonitorInit(void) {
  can_config cfg;

  // Configure CAN driver
  cfg.mcr = CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP;
  cfg.btr = CAN_BTR_SJW(0) | CAN_BTR_TS1(8) | CAN_BTR_TS2(1) | CAN_BTR_BRP(6);// | CAN_BTR_LBKM; // Add LBKM for loopback mode
  cfg.n_filters = 0;
  cfg.filters = NULL;

  // Initialize CAN driver
  can_init();
  can_start(CAND1, &cfg);

  // Start communication process
  proc_new(canMonitor_process, NULL, KERN_MINSTACKSIZE * 8, NULL);
}

static void NORETURN canMonitor_process(void) {
  encoder_can_msg_t msg;
  can_tx_frame txm;
  Timer timer_can;
  uint8_t ind = 0;

  // Initialize constant parameters of TX frame
  txm.dlc = 6;
  txm.rtr = 0;
  txm.ide = 1;
  txm.sid = 0;

  timer_setDelay(&timer_can, ms_to_ticks(1000));
  timer_setEvent(&timer_can);
  while(1) {

    timer_add(&timer_can);

    // Sending ENCODER1 and ENCODER2 data
    msg.data_f.encoder1_pos = getEncoderPosition(ENCODER1);
    msg.data_f.encoder2_pos = getEncoderPosition(ENCODER2);
    msg.data_f.encoder1_dir = getEncoderDirection(ENCODER1);
    msg.data_f.encoder2_dir = getEncoderDirection(ENCODER2);

    txm.data32[0] = msg.data32[0];
    txm.data32[1] = msg.data32[1];
    txm.eid = 100;
    can_transmit(CAND1, &txm, ms_to_ticks(10));

    // Sending ENCODER3 and ENCODER4 data
    msg.data_f.encoder1_pos = getEncoderPosition(ENCODER3);
    msg.data_f.encoder2_pos = getEncoderPosition(ENCODER4);
    msg.data_f.encoder1_dir = getEncoderDirection(ENCODER3);
    msg.data_f.encoder2_dir = getEncoderDirection(ENCODER4);

    txm.data32[0] = msg.data32[0];
    txm.data32[1] = msg.data32[1];
    txm.eid = 101;
    can_transmit(CAND1, &txm, ms_to_ticks(10));

    // Wait for the next transmission timer
    timer_waitEvent(&timer_can);
    // blink !
    if (ind == 0) {
      LED2_OFF();
      ind = 1;
    } else {
      LED2_ON();
      ind = 0;
    }
  }
}

