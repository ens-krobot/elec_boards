/*
 * Control Software for the Krobot Junior MotherBoard
 * Xavier Lagorce
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "pal.h"
#include "evtimer.h"

#include "monitor.h"
#include "can_monitor.h"

#include "lift.h"
#include "ax12.h"

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

#define ADC_GRP1_NUM_CHANNELS   8
#define ADC_GRP1_BUF_DEPTH      16

static adcsample_t samples[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];
static Thread *adctp;

const ADCConfig adccfg = {};
const ADCConversionGroup adcgrpcfg = {
  TRUE,
  ADC_GRP1_NUM_CHANNELS,
  0,
  ADC_CR2_EXTSEL_SWSTART | ADC_CR2_TSVREFE | ADC_CR2_CONT,
  0,
  0,
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),
  ADC_SQR2_SQ7_N(ADC_CHANNEL_SENSOR) | ADC_SQR2_SQ6_N(ADC_CHANNEL_VREFINT),
  ADC_SQR3_SQ5_N(ADC_CHANNEL_IN11)   | ADC_SQR3_SQ4_N(ADC_CHANNEL_IN10) |
  ADC_SQR3_SQ3_N(ADC_CHANNEL_IN11)   | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN10) |
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN11)   | ADC_SQR3_SQ0_N(ADC_CHANNEL_IN10)
};

/*
 * ADC continuous conversion thread.
 */
size_t nx = 0, ny = 0;
static void adccallback(adcsample_t *buffer, size_t n) {

  static uint16_t count = 0;

  if (buffer[11] >= 2400 ) {
    palClearPad(IOPORT3, GPIOC_LED);
    //canSetScrew(0,0,0,0,0);
    //while(1);
    count++;
  }
  else
    palSetPad(IOPORT3, GPIOC_LED);

  if (count >= 10) {
    canSetScrew(0,0,0,0,0);
    while(1);
  }
}

static WORKING_AREA(adc_continuous_wa, 256);
static msg_t adc_continuous_thread(void *p){

  (void)p;
  palSetGroupMode(IOPORT3,
                  PAL_PORT_BIT(0) | PAL_PORT_BIT(1),
                  PAL_MODE_INPUT_ANALOG);
  adcStart(&ADCD1, &adccfg);
  while(1) {
  adcStartConversion(&ADCD1, &adcgrpcfg, samples,
                     ADC_GRP1_BUF_DEPTH, adccallback);
  adcWaitConversion(&ADCD1, TIME_INFINITE);
  adcStop(&ADCD1);}
  return 0;
}


static WORKING_AREA(waThread2, 1024);
static msg_t Thread2(void *arg) {

  /*TIM_OCInitTypeDef  TIM_OCInitStructure;

  // PWM1 Mode configuration: Channel1
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  (void)arg;
  while (TRUE) {

    TIM_OCInitStructure.TIM_Pulse = 3272;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    chThdSleepMilliseconds(10000);

    TIM_OCInitStructure.TIM_Pulse = 6545;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    chThdSleepMilliseconds(10000);
    }
  };*/

  adctp = chThdCreateStatic(adc_continuous_wa, sizeof(adc_continuous_wa),
                            NORMALPRIO + 9, adc_continuous_thread, NULL);

  chThdSleep(MS2ST(85000));
  canSetScrew(0, 0, 0, 0, 0);
  while(1);

  return 0;
}


/*
 * Executed as event handler at 500mS intervals.
 */
static void TimerHandler(eventid_t id) {
  uint8_t color;

  (void)id;

  if (!palReadPad(IOPORT3, 8)) {
    if (palReadPad(IOPORT3, 6))
      color = 0;
    else
      color = 1;

    chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO+1, Thread2, NULL);


    canSetScrew(0, 0, 0, -174, 0);
    chThdSleep(MS2ST(2000));
    canSetScrew(0, 0, 0, 0, 0);
    chThdSleep(MS2ST(100));
    if (color == 0) {
      canSetScrew(0, 0, 0, 0, 40);
      chThdSleep(MS2ST(1600));
    } else
    {
      canSetScrew(0, 0, 0, 0, -40);
      chThdSleep(MS2ST(1600));
    }
    canSetScrew(0, 0, 0, 0, 0);
    chThdSleep(MS2ST(100));
    canSetScrew(0, 0, 0, -300, 0);
    chThdSleep(MS2ST(15000));
    canSetScrew(0, 0, 0, 0, 0);
  }
  /*if (!palReadPad(IOPORT1, GPIOA_BUTTON)) {

    palClearPad(IOPORT3, GPIOC_LED);
    chThdSleepMilliseconds(100);
    palSetPad(IOPORT3, GPIOC_LED);
    chThdSleepMilliseconds(100);
    palClearPad(IOPORT3, GPIOC_LED);
    chThdSleepMilliseconds(100);
    palSetPad(IOPORT3, GPIOC_LED);
    chThdSleepMilliseconds(100);
    palClearPad(IOPORT3, GPIOC_LED);
    chThdSleepMilliseconds(100);
    palSetPad(IOPORT3, GPIOC_LED);


    chThdSleepMilliseconds(2000);

    canSetScrew(0, 0, 141, 141, 0);
    chThdSleepMilliseconds(5000);

    canSetScrew(0, 0, 0, 0, 0);
    chThdSleepMilliseconds(200);

    canSetScrew(450, 200, 0, 0, 40);
    chThdSleepMilliseconds(3000);

    canSetScrew(0, 0, 0, 0, 0);
    chThdSleepMilliseconds(200);

    canSetScrew(0, 0, 0, 0, 60);
    chThdSleepMilliseconds(2000);

    canSetScrew(0, 0, 0, 0, 0);
    chThdSleepMilliseconds(200);

    canSetScrew(0, 0, 200, 0, 0);
    chThdSleepMilliseconds(1500);

    canSetScrew(0, 0, 0, 0, 0);
    chThdSleepMilliseconds(200);

    canSetScrew(0, 0, 0, 200, 0);
    chThdSleepMilliseconds(3500);

    canSetScrew(0, 0, 0, 0, 0);
    chThdSleepMilliseconds(200);

    canSetScrew(0, 0, 282, 282, 0);
    chThdSleepMilliseconds(1500);

    canSetScrew(0, 0, 0, 0, 0);
  }*/
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
   * Activates the serial driver 1 using the driver default configuration.
   */
  sdStart(&SD1, NULL);

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
   * Initialise the lift
   */
  liftInit();
  ax12Init();

  //chThdSleep(MS2ST(2000));
  
  /*
   * Init pins
   */
  palSetGroupMode(IOPORT3, PAL_PORT_BIT(6) | PAL_PORT_BIT(8), PAL_MODE_INPUT_PULLDOWN);
  palSetGroupMode(IOPORT3, PAL_PORT_BIT(0) | PAL_PORT_BIT(2), PAL_MODE_INPUT_ANALOG);
  adcStart(&ADCD1, NULL);
  //adctp = chThdCreateStatic(adc_continuous_wa, sizeof(adc_continuous_wa),
  //                          NORMALPRIO + 9, adc_continuous_thread, NULL);
  /*
   * Creates the blinker thread.
   */
  //chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO+1, Thread1, NULL);
  //chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO+1, Thread2, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and listen for events.
   */

  uint8_t params[] = {P_GOAL_POSITION, 0x00, 0x02, 0x00, 0x02};
  ax12SendPacket(1, INST_WRITE, 5, params);

  evtInit(&evt, MS2ST(500));            /* Initializes an event timer object.   */
  evtStart(&evt);                       /* Starts the event timer.              */
  chEvtRegister(&evt.et_es, &el0, 0);   /* Registers on the timer event source. */
  while (TRUE)
    chEvtDispatch(evhndl, chEvtWaitOne(ALL_EVENTS));
  return 0;
}
