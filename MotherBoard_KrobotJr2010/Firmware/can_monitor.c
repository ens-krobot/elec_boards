/*
 * CAN monitor to receive/send informations
 * Xavier Lagorce
 */

#include "can_monitor.h"

Thread *canrtp;

volatile int32_t motorSpeeds[3];
volatile uint16_t motorPositions[3];

// Thread function to receive CAN frames
static WORKING_AREA(can_rx_wa, 256);
static msg_t can_rx(void *p) {
  EventListener el;
  CANRxFrame rxmsg;
  speedMsg_t *speedMsg;

  (void)p;
  chEvtRegister(&CAND1.cd_rxfull_event, &el, 0);
  while(!chThdShouldTerminate()) {
    if (chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(100)) == 0)
      continue;
    while (canReceive(&CAND1, &rxmsg, TIME_IMMEDIATE) == RDY_OK) {
      /* Process message.*/
      palTogglePad(IOPORT3, GPIOC_LED);
      chThdSleepMilliseconds(10);
      palTogglePad(IOPORT3, GPIOC_LED);

      // Informations
      if (rxmsg.cf_RTR == CAN_RTR_DATA && rxmsg.cf_IDE == CAN_IDE_EXT) {
        switch (rxmsg.cf_EID) {
        case 0x501:
          // encoder positions
          motorPositions[0] = rxmsg.cf_data16[0];
          motorPositions[1] = rxmsg.cf_data16[1];
          motorPositions[2] = rxmsg.cf_data16[2];
          break;
        case 0x401:
          // Motor speeds
          speedMsg = (speedMsg_t*)(&(rxmsg.cf_data8[0]));
          motorSpeeds[0] = speedMsg->speeds.motor1;
          motorSpeeds[1] = speedMsg->speeds.motor2;
          motorSpeeds[2] = speedMsg->speeds.motor3;
          break;
        default:
          break;
        }
      }
    }
  }
  chEvtUnregister(&CAND1.cd_rxfull_event, &el);
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
}

int32_t canGetSpeed(uint8_t motor) {
  switch(motor) {
  case MOTOR1:
    return motorSpeeds[0];
    break;
  case MOTOR2:
    return motorSpeeds[1];
    break;
  case MOTOR3:
    return motorSpeeds[2];
    break;
  default:
    return 0;
    break;
  }
}

uint16_t canGetPosition(uint8_t motor) {
    switch(motor) {
  case MOTOR1:
    return motorPositions[0];
    break;
  case MOTOR2:
    return motorPositions[1];
    break;
  case MOTOR3:
    return motorPositions[2];
    break;
  default:
    return 0;
    break;
  }
}

void canSetSpeeds(int32_t motor1, int32_t motor2, int32_t motor3) {
  CANTxFrame txmsg;
  speedMsg_t *speedMsg;

  txmsg.cf_IDE = CAN_IDE_EXT;
  txmsg.cf_EID = 0x400;
  txmsg.cf_RTR = CAN_RTR_DATA;
  txmsg.cf_DLC = 8;
  speedMsg = (speedMsg_t*)(&(txmsg.cf_data8[0]));
  speedMsg->speeds.motor1 = motor1;
  speedMsg->speeds.motor2 = motor2;
  speedMsg->speeds.motor3 = motor3;

  canTransmit(&CAND1, &txmsg, MS2ST(100));
}

void canSetScrew(int16_t ptX, int16_t ptY, int16_t vX, int16_t vY, int16_t omega) {
  CANTxFrame txmsg;
  moveMsg_t *moveMsg;

  txmsg.cf_IDE = CAN_IDE_EXT;
  txmsg.cf_EID = 0x300;
  txmsg.cf_RTR = CAN_RTR_DATA;
  txmsg.cf_DLC = 8;
  moveMsg = (moveMsg_t*)(&(txmsg.cf_data8[0]));
  moveMsg->move.ptX   = ptX;
  moveMsg->move.ptY   = ptY;
  moveMsg->move.vX    = vX;
  moveMsg->move.vY    = vY;
  moveMsg->move.omega = omega;

  canTransmit(&CAND1, &txmsg, MS2ST(100));
}
