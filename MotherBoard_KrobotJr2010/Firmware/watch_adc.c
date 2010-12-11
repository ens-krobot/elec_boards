/*
 * Manager to watch the card ADCs
 * Xavier Lagorce
 */

#include "watch_adc.h"

#define ADC_GRP1_NUM_CHANNELS   8
#define ADC_GRP1_BUF_DEPTH      3

EventSource adcAlarmWarn[8], adcAlarmOK[8];

static adcsample_t samples[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];
static adcsample_t *curBuffer;
static Thread *adctp;

static uint16_t compHigh[8] = {4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096};
static uint16_t compLow[8]  = {0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t alarmActi[8] = {0, 0, 0, 0, 0, 0, 0, 0};

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
static void adccallback(adcsample_t *buffer, size_t n) {

  static uint8_t ind[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t i;

  (void)n;

  curBuffer = buffer;

  for (i=0; i < 8; i++) {
    if (alarmActi[i]) {
      if (ind[i] == 0 && buffer[i] >= compHigh[i]) {
        ind[i] = 1;
        chEvtBroadcastI(&adcAlarmWarn[i]);
      }
      if (ind[i] == 1 && buffer[i] <= compLow[i]) {
        ind[i] = 0;
        chEvtBroadcastI(&adcAlarmOK[i]);
      }
    }
  }
}

static WORKING_AREA(adc_continuous_wa, 256);
static msg_t adc_continuous_thread(void *p){

  (void)p;
  adcStartConversion(&ADCD1, &adcgrpcfg, samples,
                     ADC_GRP1_BUF_DEPTH, adccallback);
  adcWaitConversion(&ADCD1, TIME_INFINITE);
  return 0;
}

void adcWatchInit(void) {

  uint8_t i;

  // Init pins
  palSetGroupMode(IOPORT1, PAL_PORT_BIT(4) | PAL_PORT_BIT(5), PAL_MODE_INPUT_ANALOG);
  palSetGroupMode(IOPORT3, PAL_PORT_BIT(0) | PAL_PORT_BIT(1) | PAL_PORT_BIT(2)
                  | PAL_PORT_BIT(3) | PAL_PORT_BIT(5), PAL_MODE_INPUT_ANALOG);

  // Start ADC driver
  adcStart(&ADCD1, &adccfg);

  // Init trigger event
  for (i=0; i < 8; i++) {
    chEvtInit(&adcAlarmWarn[i]);
    chEvtInit(&adcAlarmOK[i]);
  }

  // Start conversion Thread
  adctp = chThdCreateStatic(adc_continuous_wa, sizeof(adc_continuous_wa),
                            NORMALPRIO + 9, adc_continuous_thread, NULL);
}

void adcSetAlarm(uint8_t adc, uint16_t histLow, uint16_t histHigh) {

  chSysLock();
  compLow[adc] = histLow;
  compHigh[adc] = histHigh;
  alarmActi[adc] = 1;
  chSysUnlock();
}

void adcResetAlarm(uint8_t adc) {
  alarmActi[adc] = 0;
}

adcsample_t adcGetSample(uint8_t adc) {
  if (adc > 7)
    return 0;
  return curBuffer[adc];
}
