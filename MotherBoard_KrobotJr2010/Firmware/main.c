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
#include "encoder.h"
#include "cpu_load.h"
#include "speed_control.h"
#include "trajectory.h"

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
    /*sc_setRefSpeed(MOTOR1, 0);
    sc_setRefSpeed(MOTOR2, -360);
    sc_setRefSpeed(MOTOR3, 360);
    chThdSleepMilliseconds(2000);

    sc_setRefSpeed(MOTOR1, 360);
    sc_setRefSpeed(MOTOR2, 0);
    sc_setRefSpeed(MOTOR3, -360);
    chThdSleepMilliseconds(2000);

    sc_setRefSpeed(MOTOR1, -360);
    sc_setRefSpeed(MOTOR2, 360);
    sc_setRefSpeed(MOTOR3, 0);
    chThdSleepMilliseconds(2000);

    sc_setRefSpeed(MOTOR1, 0);
    sc_setRefSpeed(MOTOR2, 0);
    sc_setRefSpeed(MOTOR3, 0);

    palClearPad(IOPORT3, GPIOC_LED);
    chThdSleepMilliseconds(1000);

    palSetPad(IOPORT3, GPIOC_LED);
    chThdSleepMilliseconds(1000);

    sc_setRefSpeed(MOTOR1, 360);
    sc_setRefSpeed(MOTOR2, 360);
    sc_setRefSpeed(MOTOR3, 360);
    chThdSleepMilliseconds(3000);

    sc_setRefSpeed(MOTOR1, 0);
    sc_setRefSpeed(MOTOR2, 0);
    sc_setRefSpeed(MOTOR3, 0);
    chThdSleepMilliseconds(3000);

    sc_setRefSpeed(MOTOR1, -360);
    sc_setRefSpeed(MOTOR2, -360);
    sc_setRefSpeed(MOTOR3, -360);
    chThdSleepMilliseconds(3000);

    sc_setRefSpeed(MOTOR1, 0);
    sc_setRefSpeed(MOTOR2, 0);
    sc_setRefSpeed(MOTOR3, 0);*/

    palClearPad(IOPORT3, GPIOC_LED);
    chThdSleepMilliseconds(1000);

    palSetPad(IOPORT3, GPIOC_LED);
    chThdSleepMilliseconds(1000);

    /*setScrew(0, 200, 0, 0, 60);
    chThdSleepMilliseconds(6000);
    turn(0);*/
  }
  return 0;
}

/*
 * Executed as event handler at 500mS intervals.
 */
static void TimerHandler(eventid_t id) {

  (void)id;
  if (palReadPad(IOPORT1, GPIOA_BUTTON)) {
    /*cputs("Coucou !\r");
    if (TIM_GetCounter(TIM3) == 0)
      cputs("c'est nul !\r");
    else
      cputs("ca marche (peut etre...)\r");
      fflush(stdout);*/
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
   * Initialise the speed controller
   */
  speedControlInit();

  chThdSleepMilliseconds(2000);


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
