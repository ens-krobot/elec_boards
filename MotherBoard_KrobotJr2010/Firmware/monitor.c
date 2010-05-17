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

void getHandler(BaseChannel *chp, int argc, char* argv[]) {
  uint16_t position;

  (void)argv;
  if (argc > 0) {
    shellPrintLine(chp, "Usage : get");
    return;
  }
  position = canGetPosition(MOTOR1);
  iprintf("motor1 :  %d%d%d%d%d\r\n", (position/10000)%10,
                                      (position/1000)%10,
                                      (position/100)%10,
                                      (position/10)%10,
                                      (position)%10);
  position = canGetPosition(MOTOR2);
  iprintf("motor2 :  %d%d%d%d%d\r\n", (position/10000)%10,
                                      (position/1000)%10,
                                      (position/100)%10,
                                      (position/10)%10,
                                      (position)%10);
  position = canGetPosition(MOTOR3);
  iprintf("motor3 :  %d%d%d%d%d\r\n", (position/10000)%10,
                                      (position/1000)%10,
                                      (position/100)%10,
                                      (position/10)%10,
                                      (position)%10);
};

void demoHandler(BaseChannel *chp, int argc, char* argv[]) {

  if (argc != 1) {    
    shellPrintLine(chp, "Usage : demo mode.");
    shellPrintLine(chp, " 1 : carré");
    shellPrintLine(chp, " 2 : tourne");
    shellPrintLine(chp, " 3 : tourne autour d'un point");
    return;
  }
  switch (argv[0][0]) {
    case '1':
      canSetScrew(0, 0, 100, 0, 0);
      chThdSleepMilliseconds(2000);
      canSetScrew(0, 0, 0, 100, 0);
      chThdSleepMilliseconds(2000);
      canSetScrew(0, 0, -100, 0, 0);
      chThdSleepMilliseconds(2000);
      canSetScrew(0, 0, 0, -100, 0);
      chThdSleepMilliseconds(2000);
      canSetScrew(0, 0, 0, 0, 0);
      break;
    case '2':
      canSetScrew(0, 0, 0, 0, 360);
      chThdSleepMilliseconds(1000);
      canSetScrew(0, 0, 0, 0, -360);
      chThdSleepMilliseconds(1000);
      canSetScrew(0, 0, 0, 0, 0);
      break;
    case '3':
      canSetScrew(0, 200, 0, 0, 60);
      chThdSleepMilliseconds(6000);
      canSetScrew(0, 0, 0, 0, 0);
      break;
  default:
      shellPrintLine(chp, "Mode inconnu");
      return;
  }
}


void getSpeedHandler(BaseChannel *chp, int argc, char* argv[]) {

  int32_t speed;

  if (argc != 1) {    
    shellPrintLine(chp, "Usage : getSpeed numMoteur.");
    return;
  }
  switch (argv[0][0]) {
    case '1':
      speed = canGetSpeed(MOTOR1);
      break;
    case '2':
      speed = canGetSpeed(MOTOR2);
      break;
    case '3':
      speed = canGetSpeed(MOTOR3);
      break;
    default:
      shellPrintLine(chp, "Mauvais moteur spécifié");
      return;
  }

  iprintf("vitesse :  %ld%ld%ld%ld%ld\r\n", (speed/10000)%10,
                                            (speed/1000)%10,
                                            (speed/100)%10,
                                            (speed/10)%10,
                                            (speed)%10);
}

void moveHandler(BaseChannel *chp, int argc, char* argv[]) {

  int16_t ptX, ptY, vX, vY, omega;
  uint16_t t;

  if (argc != 6) {    
    shellPrintLine(chp, "Usage : move ptX ptY vX vY omega t");
    return;
  }
  ptX = atoi(argv[0]);
  ptY = atoi(argv[1]);
  vX = atoi(argv[2]);
  vX = atoi(argv[3]);
  omega = atoi(argv[4]);
  t = atoi(argv[5]);

  canSetScrew(ptX, ptY, vX, vY, omega);
  chThdSleepMilliseconds(t);
}

void liftHandler(BaseChannel *chp, int argc, char* argv[]) {

  uint16_t h;

  if (argc != 1 && argc != 2) {    
    shellPrintLine(chp, "Usage : lift (up|down|move h)");
    return;
  }

  if(strcmp(argv[0], "up") == 0)
    h = LIFT_UP;
  else if(strcmp(argv[0], "down") == 0)
    h = LIFT_DOWN;
  else if(strcmp(argv[0], "move") == 0) {
    if(argc != 2) {
      shellPrintLine(chp, "Usage : lift (up|down|move h)");
      return;
    }
    h = atoi(argv[1]);
  }
  else {
    shellPrintLine(chp, "Usage : lift (up|down|move h)");
    return;
  }
  liftGoto(h);
}

/*
 * Shell configuration variables
 */
static const ShellCommand commands[] = {
  {"bonjour", bonjourHandler},
  {"get", getHandler},
  {"demo", demoHandler},
  {"getSpeed", getSpeedHandler},
  {"move", moveHandler},
  {"lift", liftHandler},
  {NULL, NULL}
};

static const ShellConfig shellConfig = {
  (BaseChannel*)&MONITOR_USART,
  commands
};

void monitorInit(void) {
  shellInit();
  shellCreate(&shellConfig, THD_WA_SIZE(512), NORMALPRIO);
  cdtp = chThdCreateFromHeap(NULL, THD_WA_SIZE(128), NORMALPRIO + 1, consoleThread, NULL);
}
