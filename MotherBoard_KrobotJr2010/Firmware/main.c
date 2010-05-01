/*
 * Control Software for the Krobot Junior MotherBoard
 * Xavier Lagorce
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "evtimer.h"

#include "monitor.h"
#include "can_monitor.h"

/*
 * Global variables
 */

/*
 * Red LEDs blinker thread, times are in milliseconds.
 */
static WORKING_AREA(waThread1, 1024);
static msg_t Thread1(void *arg) {

  (void)arg;
  while (TRUE) {

    palClearPad(IOPORT3, GPIOC_LED);
    chThdSleepMilliseconds(1000);

    palSetPad(IOPORT3, GPIOC_LED);
    chThdSleepMilliseconds(1000);
  }
  return 0;
}

/*
 * Executed as event handler at 500mS intervals.
 */
static void TimerHandler(eventid_t id) {
  CANTxFrame txmsg;
  moveMsg_t *moveMsg;

  (void)id;
  if (palReadPad(IOPORT1, GPIOA_BUTTON)) {

  txmsg.cf_IDE = CAN_IDE_EXT;
  txmsg.cf_EID = 0x300;
  txmsg.cf_RTR = CAN_RTR_DATA;
  txmsg.cf_DLC = 8;
  moveMsg = (moveMsg_t*)(&(txmsg.cf_data8[0]));
  moveMsg->move.ptX   = 0;
  moveMsg->move.ptY   = 200;
  moveMsg->move.vX    = 0;
  moveMsg->move.vY    = 0;
  moveMsg->move.omega = 60;

  canTransmit(&CAND1, &txmsg, MS2ST(100));
  }
}

/*
 * Entry point, note, the main() function is already a thread in the system
 * on entry.
 */
int main(int argc, char **argv) {

  static const evhandler_t evhndl[] = {
    TimerHandler,
  };
  static EvTimer evt;
  struct EventListener el0;

  (void)argc;
  (void)argv;

  /*
   * Activates the serial driver 2 using the driver default configuration.
   */
  sdStart(&SD2, NULL);

  /*
   * Initialise the monitor
   */
  monitorInit();

  /*
   * Initialise the CAN monitor
   */
  canMonitorInit();
  canMonitorStart();

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO+1, Thread1, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and listen for events.
   */
  evtInit(&evt, MS2ST(500));            /* Initializes an event timer object.   */
  evtStart(&evt);                       /* Starts the event timer.              */
  chEvtRegister(&evt.et_es, &el0, 0);   /* Registers on the timer event source. */
  while (TRUE)
    chEvtDispatch(evhndl, chEvtWaitOne(ALL_EVENTS));
  return 0;
}
