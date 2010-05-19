/*
 * Serial communication with AX12 digital servomotors
 * Xavier Lagorce
 */

#include "ax12.h"

BaseChannel *ax12_chp = (BaseChannel*)&SD2;

typedef enum {
  RD_READY = 0,
  RD_BEGIN = 1,
  RD_ID = 2,
  RD_LEN = 3,
  RD_ERROR = 4,
  RD_PAYLOAD = 5,
  RD_CHECKSUM = 6,
} rdstate_t;


static WORKING_AREA(waThreadAX12Rec, 256);
static msg_t ThreadAX12Rec(void *arg) {

  rdstate_t state = RD_READY;
  uint8_t id=0, len=0, count=0, error=0, chksum=0, i;
  uint8_t payload[10];
  Thread *waiter = NULL, *msg = NULL;

  (void)arg;
  while (TRUE) {
    uint8_t c = (uint8_t)chIOGet(ax12_chp);

    msg = (Thread*)chMsgGet();
    if (msg != NULL) {
      waiter = msg;
      chMsgRelease(RDY_OK);
    }

    switch (state) {
    case RD_READY:
      if (c == 0xFF)
        state = RD_BEGIN;
      break;
    case RD_BEGIN:
      if (c == 0xFF)
        state = RD_ID;
      else
        state = RD_READY;
      break;
    case RD_ID:
      id = c;
      state = RD_LEN;
      break;
    case RD_LEN:
      len = c - 2;
      count = 0;
      state = RD_ERROR;
      break;
    case RD_ERROR:
      error = c;
      if (len == 0)
        state = RD_CHECKSUM;
      else
        state = RD_PAYLOAD;
      break;
    case RD_PAYLOAD:
      count++;
      payload[count] = c;
      if (count == len)
        state = RD_CHECKSUM;
      break;
    case RD_CHECKSUM:
      chksum = id+len+error;
      for (i=0; i < len ; i++)
        chksum += payload[i];
      chksum = ~chksum;
      if (waiter != NULL) {
        if (c != chksum) {
          chEvtSignal(waiter, EVT_AX12_ERROR);
        }
        else {
          chEvtSignal(waiter, EVT_AX12_PACKET);
          chMsgSend(waiter, (msg_t)payload);
        }
        waiter = NULL;
      }
      state = RD_READY;
      break;
    default:
      state = RD_READY;
      break;
    }
  }
  return 0;
}

void ax12Init(void) {
  const SerialConfig ax12_config =
  {
    AX12_BAUDRATE,
    0,
    USART_CR2_STOP1_BITS | USART_CR2_LINEN,
    0
  };

  palSetPadMode(IOPORT1, 1, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPad(IOPORT1, 1); // reading mode

  sdStart(&SD2, &ax12_config);
  chIOPut(ax12_chp, 0xFF);
  //chThdCreateStatic(waThreadAX12Rec, sizeof(waThreadAX12Rec), NORMALPRIO+1, ThreadAX12Rec, NULL);
}

void ax12SendPacket(uint8_t id, uint8_t instruction, uint8_t len, uint8_t *params) {
  uint8_t chksum, i;
  uint64_t j;

  chksum = id + len + 2 + instruction;
  for (i = 0; i < len; i++)
    chksum += params[i];
  chksum = ~chksum;

  chSysLock();

  chIOPut(ax12_chp, 0xFF);
  chIOPut(ax12_chp, 0xFF);
  chIOPut(ax12_chp, id);
  chIOPut(ax12_chp, len+2);
  chIOPut(ax12_chp, instruction);
  for (i=0; i<len; i++)
    chIOPut(ax12_chp, params[i]);
  chIOPut(ax12_chp, chksum);

  chSysUnlock();
}

void ax12Configure(uint8_t old_id, uint8_t new_id) {

  uint8_t paramsDelay[] = {P_RETURN_DELAY_TIME, 0x0A};
  uint8_t paramsID[]    = {P_ID, new_id};

  // UNSAFE !!
  /*
  // Reset default settings
  ax12SendPacket(ID_BROADCAST, INST_RESET, 0, NULL);
  chThdSleepMilliseconds(1);
  // Set response delay at maximum
  ax12SendPacket(ID_BROADCAST, INST_WRITE, sizeof(paramsDelay), paramsDelay);
  chThdSleepMilliseconds(1);
  */
  // Set ID
  ax12SendPacket(old_id, INST_WRITE, sizeof(paramsID), paramsID);
  chThdSleepMilliseconds(1);

}

void ax12Goto(uint8_t id, uint16_t position, uint16_t speed, uint8_t mode) {

  uint8_t paramsGoto[] = {P_GOAL_POSITION, position%256, position/256, speed%256, speed/256};

  if (mode == CMD_ACTION)
    ax12SendPacket(id, INST_REG_WRITE, sizeof(paramsGoto), paramsGoto);
  else
    ax12SendPacket(id, INST_WRITE, sizeof(paramsGoto), paramsGoto);
}

void ax12Action(uint8_t id) {

  ax12SendPacket(id, INST_ACTION, 0, NULL);
}

void ax12Ping(uint8_t id) {

  ax12SendPacket(id, INST_PING, 0, NULL);
}
