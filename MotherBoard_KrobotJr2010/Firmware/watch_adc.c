/*
 * Manager to watch the card ADCs
 * Xavier Lagorce
 */

#include "watch_adc.h"

#define ADC_GRP1_NUM_CHANNELS   8
#define ADC_GRP1_BUF_DEPTH     16

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
  ADC_SQR2_SQ7_N(ADC_CHANNEL_IN5) | ADC_SQR2_SQ6_N(ADC_CHANNEL_IN4),
  ADC_SQR3_SQ5_N(ADC_CHANNEL_IN15)   | ADC_SQR3_SQ4_N(ADC_CHANNEL_IN14) |
  ADC_SQR3_SQ3_N(ADC_CHANNEL_IN13)   | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN12) |
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN11)   | ADC_SQR3_SQ0_N(ADC_CHANNEL_SENSOR)
};

/*
 * ADC continuous conversion thread.
 */
size_t nx = 0, ny = 0;
static void adccallback(adcsample_t *buffer, size_t n) {

  if (buffer[ADC_3] >= 2000)
    palClearPad(IOPORT3, GPIOC_LED);
  else
    palSetPad(IOPORT3, GPIOC_LED);
}

static WORKING_AREA(adc_continuous_wa, 2048);
static msg_t adc_continuous_thread(void *p){

  (void)p;
  while(1) {
    adcStartConversion(&ADCD1, &adcgrpcfg, samples,
                       ADC_GRP1_BUF_DEPTH, adccallback);
    adcWaitConversion(&ADCD1, TIME_INFINITE);
  }
  return 0;
}

void adcWatchInit(void) {

  // Init pins
  palSetGroupMode(IOPORT1, PAL_PORT_BIT(4) | PAL_PORT_BIT(5), PAL_MODE_INPUT_ANALOG);
  palSetGroupMode(IOPORT3, PAL_PORT_BIT(0) | PAL_PORT_BIT(1) | PAL_PORT_BIT(2)
                  | PAL_PORT_BIT(3) | PAL_PORT_BIT(5), PAL_MODE_INPUT_ANALOG);

  // Start ADC driver
  adcStart(&ADCD1, &adccfg);

  // Start conversion Thread
  adctp = chThdCreateStatic(adc_continuous_wa, sizeof(adc_continuous_wa),
                            NORMALPRIO + 9, adc_continuous_thread, NULL);
}
