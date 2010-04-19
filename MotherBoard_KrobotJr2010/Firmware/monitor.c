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

void resetHandler(BaseChannel *chp, int argc, char* argv[]) {

  if (argc != 1) {
    shellPrintLine(chp, "Usage : reset numEncodeur.");
    return;
  }
  switch (argv[0][0]) {
    case '1':
      resetEncoderPosition(ENCODER1);
      shellPrintLine(chp, "Reset encodeur 1.");
      break;
    case '2':
      resetEncoderPosition(ENCODER2);
      shellPrintLine(chp, "Reset encodeur 2.");
      break;
    case '3':
      resetEncoderPosition(ENCODER3);
      shellPrintLine(chp, "Reset encodeur 3.");
      break;
    case 'a':
      resetEncoderPosition(ENCODER1);
      resetEncoderPosition(ENCODER2);
      resetEncoderPosition(ENCODER3);
      shellPrintLine(chp, "Reset de tous les encodeurs.");
      break;
    default:
      shellPrintLine(chp, "Il n'y a que 3 encodeurs !");      
  }
}

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
  motorSetSpeed(motor, speed);
  iprintf("set speed :  %d%d%d%d%d\r\n", (speed/10000)%10,
                                         (speed/1000)%10,
                                         (speed/100)%10,
                                         (speed/10)%10,
                                         (speed)%10);
}

void motorEnableHandler(BaseChannel *chp, int argc, char* argv[]) {

  uint8_t motor = 0;

  if (argc != 2) {    
    shellPrintLine(chp, "Usage : motor numMoteur enabled.");
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
  switch (argv[1][0]) {
    case '0':
      disableMotor(motor);
      break;
    case '1':
      enableMotor(motor);
      break;
    default:
      shellPrintLine(chp, "gné ?");
      return;
  }
}


/*
 * Shell configuration variables
 */
static const ShellCommand commands[] = {
  {"bonjour", bonjourHandler},
  {"load", loadHandler},
  {"get", getHandler},
  {"reset", resetHandler},
  {"setSpeed", setSpeedHandler},
  {"motorEnable", motorEnableHandler},
  {NULL, NULL}
};

static const ShellConfig shellConfig = {
  (BaseChannel*)&MONITOR_USART,
  commands
};

void monitorInit(void) {
  shellInit();
  shellCreate(&shellConfig, THD_WA_SIZE(512), NORMALPRIO);
  cdtp = chThdCreateFromHeap(NULL, THD_WA_SIZE(512), NORMALPRIO + 1, consoleThread, NULL);
}
