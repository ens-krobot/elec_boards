/*
 * CAN monitor to receive/send informations
 * Xavier Lagorce
 */

#include "can_monitor.h"

Thread *canrtp;
Thread *canttp;


// Thread function to receive CAN frames
static WORKING_AREA(can_rx_wa, 256);
static msg_t can_rx(void *p) {
  EventListener el;
  CANRxFrame rxmsg;
  CANTxFrame txmsg;
  uint8_t transmit;
  speedMsg_t *speedMsg;
  moveMsg_t *moveMsg;

  (void)p;

  txmsg.cf_IDE = CAN_IDE_EXT;
  txmsg.cf_RTR = CAN_RTR_DATA;

  chEvtRegister(&CAND1.cd_rxfull_event, &el, 0);
  while(!chThdShouldTerminate()) {
    if (chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(100)) == 0)
      continue;
    while (canReceive(&CAND1, &rxmsg, TIME_IMMEDIATE) == RDY_OK) {
      /* Process message.*/
      //palTogglePad(IOPORT3, GPIOC_LED);

      transmit = FALSE;
      // Ask for info
      if (rxmsg.cf_RTR == CAN_RTR_REMOTE && rxmsg.cf_IDE == CAN_IDE_EXT) {
        switch (rxmsg.cf_EID) {
        case 0x501:
          // encoder positions
          txmsg.cf_EID = 0x501;
          txmsg.cf_DLC = 6;
          txmsg.cf_data16[0] = getEncoderPosition(ENCODER1);
          txmsg.cf_data16[1] = getEncoderPosition(ENCODER2);
          txmsg.cf_data16[2] = getEncoderPosition(ENCODER3);
          transmit = TRUE;
          break;
        case 0x401:
          // Motor speeds
          txmsg.cf_EID = 0x401;
          txmsg.cf_DLC = 8;
          speedMsg = (speedMsg_t*)(&(txmsg.cf_data8[0]));
          speedMsg->speeds.motor1=sc_getRealSpeed(MOTOR1);
          speedMsg->speeds.motor2=sc_getRealSpeed(MOTOR1);
          speedMsg->speeds.motor3=sc_getRealSpeed(MOTOR1);
          transmit = TRUE;
          break;
        default:
          break;
        }
      }
      // Command received
      if (rxmsg.cf_RTR == CAN_RTR_DATA && rxmsg.cf_IDE == CAN_IDE_EXT) {
        switch (rxmsg.cf_EID) {
        case 0x400:
          // Set all 3 speeds
          speedMsg = (speedMsg_t*)(&(rxmsg.cf_data8[0]));
          sc_setRefSpeed(MOTOR1, speedMsg->speeds.motor1);
          sc_setRefSpeed(MOTOR2, speedMsg->speeds.motor2);
          sc_setRefSpeed(MOTOR3, speedMsg->speeds.motor3);
          txmsg.cf_EID = 0x401;
          txmsg.cf_DLC = 8;
          txmsg.cf_data32[0] = rxmsg.cf_data32[0];
          txmsg.cf_data32[1] = rxmsg.cf_data32[1];
          transmit = TRUE;
          break;
        case 0x300:
          // Set speed screw
          moveMsg = (moveMsg_t*)(&(rxmsg.cf_data8[0]));
          setScrew(moveMsg->move.ptX, moveMsg->move.ptY, moveMsg->move.vX, moveMsg->move.vY, moveMsg->move.omega);
          txmsg.cf_EID = 0x301;
          txmsg.cf_DLC = 8;
          txmsg.cf_data32[0] = rxmsg.cf_data32[0];
          txmsg.cf_data32[1] = rxmsg.cf_data32[1];
          transmit = TRUE;
          break;
        default:
          break;
        }
      }

      if (transmit == TRUE) {
        canTransmit(&CAND1, &txmsg, MS2ST(100));
      }
    }
  }
  chEvtUnregister(&CAND1.cd_rxfull_event, &el);
  return 0;
}

// Thread function to send CAN frames
static WORKING_AREA(can_tx_wa, 256);
static msg_t can_tx(void * p) {
  CANTxFrame txmsg;
  speedMsg_t *speedMsg;

  (void)p;
  txmsg.cf_IDE = CAN_IDE_EXT;
  txmsg.cf_RTR = CAN_RTR_DATA;
  txmsg.cf_DLC = 8;

  while (!chThdShouldTerminate()) {

    txmsg.cf_EID = 0x501;
    txmsg.cf_DLC = 6;
    txmsg.cf_data16[0] = getEncoderPosition(ENCODER1);
    txmsg.cf_data16[1] = getEncoderPosition(ENCODER2);
    txmsg.cf_data16[2] = getEncoderPosition(ENCODER3);
    canTransmit(&CAND1, &txmsg, MS2ST(0));

    chThdSleepMilliseconds(10);

    txmsg.cf_EID = 0x401;
    txmsg.cf_DLC = 8;
    speedMsg = (speedMsg_t*)(&(txmsg.cf_data8[0]));
    speedMsg->speeds.motor1=sc_getRealSpeed(MOTOR1);
    speedMsg->speeds.motor2=sc_getRealSpeed(MOTOR1);
    speedMsg->speeds.motor3=sc_getRealSpeed(MOTOR1);
    canTransmit(&CAND1, &txmsg, MS2ST(0));

    chThdSleepMilliseconds(10);
  }
  return 0;
}


void canMonitorInit(void) {

  GPIO_InitTypeDef GPIO_InitStructure;

  // Low level inits
  // GPIO clock enable
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);

  // CAN1 Periph clock enable
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

  //Configure CAN pin: RX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  // Configure CAN pin: TX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  // Remap the CAN controller to the correct pins
  GPIO_PinRemapConfig(GPIO_Remap1_CAN1 , ENABLE);
}

void canMonitorStart(void) {

  // ChibiOS CAN driver init
  CANConfig cancfg = {
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    CAN_BTR_SJW(0) | CAN_BTR_TS2(1) |
    CAN_BTR_TS1(8) | CAN_BTR_BRP(6),
    0,
    NULL
  };

  canStart(&CAND1, &cancfg);
  canrtp = chThdCreateStatic(can_rx_wa, sizeof(can_rx_wa),
                             NORMALPRIO + 7, can_rx, NULL);
  canttp = chThdCreateStatic(can_tx_wa, sizeof(can_tx_wa),
                             NORMALPRIO + 7, can_tx, NULL);
}
