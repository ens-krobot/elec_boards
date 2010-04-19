/*
 * CPU Load calculator
 * Xavier Lagorce
 */

#include "cpu_load.h"
#include "monitor.h"

volatile uint16_t CPUload = 0;
volatile uint32_t counter = 0, max_value = 0;

static uint8_t loadInit = 0;

static Thread *tp = NULL;

/*
 * Thread to calculate CPU load value
 */
static msg_t ThreadLoad(void *arg) {

  (void)arg;
  systime_t time;

  time = chTimeNow();
  time += MS2ST(1000);

  max_value = 0;
  while (max_value == 0) {
    counter = 0;
    chThdSleepUntil(time);
    time += MS2ST(1000);
    if (loadInit == 0)
      loadInit = 1;
    else
      max_value = counter;
  }

  chSchReadyI(tp);

  while (TRUE) {
    counter = 0;
    time += MS2ST(1000);
    chThdSleepUntil(time);
    CPUload = (uint16_t)((max_value - counter)*1000/max_value);
  }
  return 0;
}

/*
 * Thread to increment load counter
 */
static msg_t ThreadCount(void *arg) {

  (void)arg;
  while(TRUE)
    counter++;

  return 0;
}

void CPULoadInit(void) {

  tp = chThdSelf();

  chThdCreateFromHeap(NULL, 256, LOWPRIO+1, ThreadCount, NULL);
  chThdCreateFromHeap(NULL, 256, HIGHPRIO, ThreadLoad, NULL);

  chSchGoSleepS(PRSUSPENDED);
}
