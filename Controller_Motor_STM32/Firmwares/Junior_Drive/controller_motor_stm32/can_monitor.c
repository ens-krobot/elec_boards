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
switch_status end_courses_msg;
//adc_values adc1_msg, adc2_msg;

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
  ax12_goto ax1_msg,ax2_msg;
  can_tx_frame txm;
  Timer timer_can;

  // Initialize constant parameters of TX frame
  txm.dlc = 8;
  txm.rtr = 0;
  txm.ide = 1;
  txm.sid = 0;
  
  

  timer_setDelay(&timer_can, ms_to_ticks(25));
  timer_setEvent(&timer_can);
  while(1) {

    timer_add(&timer_can);
    // Update AX12 parameters
	  ax1_msg.p.address = ax1.address;
	  ax1_msg.p.position = ax1.position;
	  ax1_msg.p.speed = ax1.speed;
	  
	  ax2_msg.p.address = ax2.address;
	  ax2_msg.p.position = ax2.position;
	  ax2_msg.p.speed = ax2.speed;
	  
    // Sending AX1 data
    txm.data32[0]=ax1_msg.d[0];
    txm.data32[1]=ax1_msg.d[1];
    txm.eid = CAN_AX12_GOTO;
    can_transmit(CAND1, &txm, ms_to_ticks(10));
    
    timer_delay(10);
    // Sending AX2 data
    txm.data32[0]=ax2_msg.d[0];
    txm.data32[1]=ax2_msg.d[1];
    txm.eid = CAN_AX12_GOTO;
    can_transmit(CAND1, &txm, ms_to_ticks(10));
    
/*
    // Sending ENCODER3 and ENCODER4 data
    msg_enc.data.encoder1_pos = getEncoderPosition(ENCODER3);
    msg_enc.data.encoder2_pos = getEncoderPosition(ENCODER4);
    msg_enc.data.encoder1_dir = getEncoderDirection(ENCODER3);
    msg_enc.data.encoder2_dir = getEncoderDirection(ENCODER4);

    txm.data32[0] = msg_enc.data32[0];
    txm.data32[1] = msg_enc.data32[1];
    txm.eid = CAN_MSG_ENCODERS34;
    can_transmit(CAND1, &txm, ms_to_ticks(10));
*/
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

    //switch_status end_courses_msg;
    //adc_values adc1_msg, adc2_msg;
    lift_cmd_can_msg_t lift_cmd_msg;

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
            break;
          case CAN_ADC_VALUES_1:
          	adc1_msg.d[0] = frame.data32[0];
          	adc1_msg.d[1] = frame.data32[1];
          	
          break;
          case CAN_ADC_VALUES_2:
          	adc2_msg.d[0] = frame.data32[0];
          	adc2_msg.d[1] = frame.data32[1];
		//if you want to test SHARP sensors
          	//if(adc2_msg.p.val1>2000) LED2_ON();
          	//else LED2_OFF();
          break;
          }
        }
      }
    }
}
