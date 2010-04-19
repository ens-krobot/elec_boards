/*
 * Monitor on USART using ChibiOS/RT Shell
 * Xavier Lagorce
 */
#include "monitor.h"

/*
 * Global variables
 */
Thread *cdtp;

/*
 * Thread to print text into the monitor in a thread safe way
 */
static msg_t consoleThread(void* arg) {
  (void)arg;
  while (!chThdShouldTerminate()) {
    puts((char*)chMsgWait());
    fflush(stdout);
    chMsgRelease(RDY_OK);
  }
  return 0;
}

/*
 * Handler functions for monitor commands
 */
void bonjourHandler(BaseChannel *chp, int argc, char* argv[]) {
  (void)argv;
  if (argc > 0) {
    shellPrintLine(chp, "Usage : bonjour");
    return;
  }
  shellPrintLine(chp, "Bonjour aussi !");
}

void loadHandler(BaseChannel *chp, int argc, char* argv[]) {
  uint16_t load;

  (void)argv;
  if (argc > 0) {    
    shellPrintLine(chp, "Usage : load.");
    return;
  }
  load = CPUload;
  iprintf("charge CPU :  %d%d%d%d%d\r\n", (load/10000)%10,
                                          (load/1000)%10,
                                          (load/100)%10,
                                          (load/10)%10,
                                          (load)%10);
}

void getHandler(BaseChannel *chp, int argc, char* argv[]) {
  uint16_t position;

  (void)argv;
  if (argc > 0) {
    shellPrintLine(chp, "Usage : get");
    return;
  }
  position = getEncoderPosition(ENCODER1);
  iprintf("encoder1 :  %d%d%d%d%d\r\n", (position/10000)%10,
                                        (position/1000)%10,
                                        (position/100)%10,
                                        (position/10)%10,
                                        (position)%10);
  position = getEncoderPosition(ENCODER2);
  iprintf("encoder2 :  %d%d%d%d%d\r\n", (position/10000)%10,
                                        (position/1000)%10,
                                        (position/100)%10,
                                        (position/10)%10,
                                        (position)%10);
  position = getEncoderPosition(ENCODER3);
  iprintf("encoder3 :  %d%d%d%d%d\r\n", (position/10000)%10,
                                        (position/1000)%10,
                                        (position/100)%10,
                                        (position/10)%10,
                                        (position)%10);
};

void setSpeedHandler(BaseChannel *chp, int argc, char* argv[]) {

  uint8_t motor = 0, i;
  int speed;

  if (argc != 2) {    
    shellPrintLine(chp, "Usage : setSpeed numMoteur vitesse.");
    return;
  }
  switch (argv[0][0]) {
    case '1':
      motor = MOTOR1;
      break;
    case '2':
      motor = MOTOR2;
      break;
    case '3':
      motor = MOTOR3;
      break;
    case 'a':
      motor = MOTOR1 | MOTOR2 | MOTOR3;
      break;
    default:
      shellPrintLine(chp, "Mauvais moteur spécifié");
      return;
  }

  for (i=0, speed=0; argv[1][i] != 0; i++) {
    if (argv[1][i] >= '0' && argv[1][i] <= '9')
      speed = speed*10 + (argv[1][i] - '0');
  }
  if (argv[1][0] == '-')
    speed = -speed;
  sc_setRefSpeed(motor, speed);
  iprintf("set speed :  %d%d%d%d%d\r\n", (speed/10000)%10,
                                         (speed/1000)%10,
                                         (speed/100)%10,
                                         (speed/10)%10,
                                         (speed)%10);
}

void getSpeedHandler(BaseChannel *chp, int argc, char* argv[]) {

  int speed;

  if (argc != 1) {    
    shellPrintLine(chp, "Usage : getSpeed numMoteur.");
    return;
  }
  switch (argv[0][0]) {
    case '1':
      speed = sc_getRealSpeed(MOTOR1);
      break;
    case '2':
      speed = sc_getRealSpeed(MOTOR2);
      break;
    case '3':
      speed = sc_getRealSpeed(MOTOR3);
      break;
    default:
      shellPrintLine(chp, "Mauvais moteur spécifié");
      return;
  }

  iprintf("vitesse :  %d%d%d%d%d\r\n", (speed/10000)%10,
                                       (speed/1000)%10,
                                       (speed/100)%10,
                                       (speed/10)%10,
                                       (speed)%10);
}


/*
 * Shell configuration variables
 */
static const ShellCommand commands[] = {
  {"bonjour", bonjourHandler},
  {"load", loadHandler},
  {"get", getHandler},
  {"setSpeed", setSpeedHandler},
  {"getSpeed", getSpeedHandler},
  {NULL, NULL}
};

static const ShellConfig shellConfig = {
  (BaseChannel*)&MONITOR_USART,
  commands
};

void monitorInit(void) {
  shellInit();
  shellCreate(&shellConfig, THD_WA_SIZE(256), NORMALPRIO);
  cdtp = chThdCreateFromHeap(NULL, THD_WA_SIZE(128), NORMALPRIO + 1, consoleThread, NULL);
}
