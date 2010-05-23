/*
 * Manager to watch the card ADCs
 * Xavier Lagorce
 */

#ifndef HEADER__WATCH_ADC
#define HEADER__WATCH_ADC

#include "ch.h"
#include "hal.h"
#include "pal.h"

#define ADC_TEMP    0
#define ADC_1       1
#define ADC_2       2
#define ADC_3       3
#define ADC_4       4
#define ADC_5       5
#define ADC_6       6
#define ADC_7       7

// Event sources
EventSource adcAlarmWarn[8], adcAlarmOK[8];

void adcWatchInit(void);
void adcSetAlarm(uint8_t adc, uint16_t histLow, uint16_t histHigh);
void adcResetAlarm(uint8_t adc);

#endif
