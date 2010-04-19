/*
  Template program using ChibiOS/RT
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "evtimer.h"

#include "monitor.h"
#include "encoder.h"
#include "motor.h"
#include "cpu_load.h"

/*
 * Global variables
 */

/*
 * Red LEDs blinker thread, times are in milliseconds.
 */
static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {

  (void)arg;
  while (TRUE) {
    palClearPad(IOPORT3, GPIOC_LED);
    chThdSleepMilliseconds(500);
    palSetPad(IOPORT3, GPIOC_LED);
    chThdSleepMilliseconds(500);
  }
  return 0;
}

/*
 * Executed as event handler at 500mS intervals.
 */
static void TimerHandler(eventid_t id) {

  (void)id;
  if (palReadPad(IOPORT1, GPIOA_BUTTON)) {
    cputs("Coucou !\r");
    if (TIM_GetCounter(TIM3) == 0)
      cputs("c'est nul !\r");
    else
      cputs("ca marche (peut etre...)\r");
    fflush(stdout);
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
   * ChibiOS/RT init
   */
  halInit();
  chSysInit();

  /*
   * Initialise the load calculator
   */
  //CPULoadInit(); // currently renders the system unstable

  /*
   * Activates the serial driver 2 using the driver default configuration.
   */
  sdStart(&SD2, NULL);

  /*
   * Initialise the monitor
   */
  monitorInit();

  /*
   * Initialise the encoder Interface
   */
  encodersInit();

  /*
   * Initialise the motor Interface
   */
  motorsInit();

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

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
