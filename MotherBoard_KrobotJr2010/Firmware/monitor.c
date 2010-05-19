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
 * Handmade atoi, because the one from standard lib does not work
 */
int32_t atoi_h(char *str) {
  int32_t res = 0;
  uint8_t neg = 0;

  while(isspace((int)*str))
    str++;

  switch(*str) {
    case '-':
      neg = 1;
    case '+':
      str++;
      break;
  }

  for(; isdigit((int)*str); str++) {
    res *= 10;
    res += *str - '0';
  }

  return neg? -res : res;
}

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
  ptX = atoi_h(argv[0]);
  ptY = atoi_h(argv[1]);
  vX = atoi_h(argv[2]);
  vY = atoi_h(argv[3]);
  omega = atoi_h(argv[4]);
  t = atoi_h(argv[5]);

  canSetScrew(ptX, ptY, vX, vY, omega);
  chThdSleepMilliseconds(t);
  canSetScrew(0, 0, 0, 0, 0);
}

void liftHandler(BaseChannel *chp, int argc, char* argv[]) {

  uint16_t h;

  if (argc != 1) {    
    shellPrintLine(chp, "Usage : lift (up|down|h)");
    return;
  }

  if(strcmp(argv[0], "up") == 0)
    h = LIFT_UP;
  else if(strcmp(argv[0], "down") == 0)
    h = LIFT_DOWN;
  else
    h = atoi_h(argv[1]);
  
  liftGoto(h);
}

void ax12Handler(BaseChannel *chp, int argc, char* argv[]) {

  uint8_t id, new_id;
  uint16_t pos, spd;

  if (argc < 2) {    
    shellPrintLine(chp, "Usage : ax12 (config|goto|goto_delayed|action) id ...");
    return;
  }

  id = atoi_h(argv[1]);

  if(strcmp(argv[0], "config") == 0) {
    if (argc != 3) {
        shellPrintLine(chp, "Usage : ax12 config old_id new_id");
        return;
    }
    new_id = atoi_h(argv[2]);
    ax12Configure(id, new_id);
  }
  else if(strcmp(argv[0], "goto") == 0) {
    if (argc != 4) {
        shellPrintLine(chp, "Usage : ax12 goto id pos speed");
        return;
    }
    pos = atoi_h(argv[2]);
    spd = atoi_h(argv[3]);
    ax12Goto(id, pos, spd, CMD_NOW);
  }
  else if(strcmp(argv[0], "goto_delayed") == 0) {
    if (argc != 4) {
        shellPrintLine(chp, "Usage : ax12 goto_delayed id pos speed");
        return;
    }
    pos = atoi_h(argv[2]);
    spd = atoi_h(argv[3]);
    ax12Goto(id, pos, spd, CMD_ACTION);
  }
  else if(strcmp(argv[0], "action") == 0) {
    ax12Action(id);
  }
  else {
    shellPrintLine(chp, "Usage : ax12 (config|goto|goto_delayed|action) ...");
    return;
  }
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
  {"ax12", ax12Handler},
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
