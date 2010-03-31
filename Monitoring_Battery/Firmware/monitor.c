/**
 * @file monitor.c
*/

#ifndef MONITOR_C
#define MONITOR_C

volatile int Measure[10] = {0};

#include "monitor.h"

void interruptMonitor(void) {
    static char state = 0;
    static char channel = 0;

    switch (state) {
        case 0:
            // Démarre la convertion
            switch (channel) {
                // Attention : ADC_CHx != x (voir la définition de SelChanConvADC et de ADC_CHx)
                case 0: SelChanConvADC(ADC_CH0); break;
                case 1: SelChanConvADC(ADC_CH1); break;
                case 2: SelChanConvADC(ADC_CH2); break;
                case 3: SelChanConvADC(ADC_CH3); break;
                case 4: SelChanConvADC(ADC_CH4); break;
                case 5: SelChanConvADC(ADC_CH5); break;
                case 6: SelChanConvADC(ADC_CH6); break;
                case 7: SelChanConvADC(ADC_CH7); break;
                case 8: SelChanConvADC(ADC_CH8); break;
                case 9: SelChanConvADC(ADC_CH9); break;
                default: break;
            }

            state = 1;
        break;

        case 1:
            // Récupère le résultat de la conversion
            if (!BusyADC()) {
                Measure[channel] = ReadADC();

                if (channel == 9)
                    channel = 0;
                else
                    channel++;

                state = 0;
            }
        break;

        default:
            state = 0;
    }
}

void initMonitor(void) {
    TRISC&= 0b11111011;
    PORTC&= 0b11111011;

    OpenADC(ADC_FOSC_64           /* A/D clock source (forcément Fosc / 64 car PIC à 48 MHz) */
        & ADC_RIGHT_JUST          /* A/D result justification */
        & ADC_20_TAD,             /* A/D acquisition time select (pas la peine d'attendre avant le */
                                  /* début de la conversion car l'acquisition a largement eu le */
                                  /* temps de se faire entre chaque interruption, toutes les 1 ms) */
        ADC_INT_OFF               /* A/D Interrupts */
        & ADC_REF_VDD_VSS,        /* A/D voltage configuration */
        ADC_10ANA                 // analog: AN0->9   digital: AN10->15
    );
}

/**
 * Donne la tension aux bornes d'une cellule de la batterie.
 *
 * @param        cell        Numéro de la cellule (de 0 à 7)
 *
 * @return       int         Tension, en mV
*/
int getCellVoltage(char cell) {
    return (int) ((((float) Measure[cell]) * 5.0 / 1023.0) * 1000.0);
}

/**
 * Donne la valeur instantanée du courant débité par la batterie.
 *
 * @return       long        Courant, en mA
*/
long getCurrent() {
    return (long) (((((float) Measure[8]) * 5.0 / 1023.0 - 2.5) * 1000.0) * CURRENT_GAIN);
}

/**
 * Récupère l'état de l'alimentation de puissance.
 *
 * @return       State       Renvoie #TRUE si la puissance est présente, #FALSE sinon
*/
BOOL getPowerState() {
    // 400 correspond à environ 2 V, soit 12 V avant le pont diviseur de tension.
    return (Measure[9] > 400);
}

/**
 * Récupère l'état de la batterie.
 *
 * @return       State       Etat : 0 = cellule(s) non connectée(s) @n
 *                                  1 = charge faible @n
 *                                  2 = charge moyenne @n
 *                                  3 = pleine charge
*/
BYTE getBatteryState(void) {
    int i, cellVoltage;
    BYTE charge = 3;

    for (i = 0; i < NB_CELLS; i++) {
        cellVoltage = getCellVoltage(i);

        if (cellVoltage < FULL_CHARGE_THRES && charge > 2)
            charge = 2;

        if (cellVoltage < LOW_CHARGE_THRES && charge > 1)
            charge = 1;

        if (cellVoltage < ABSENT_CELL_THRES) {
            charge = 0;
            break;
        }
    }

    return charge;
}

#endif
