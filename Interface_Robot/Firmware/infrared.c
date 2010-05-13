#ifndef INFRARED_C
#define INFRARED_C

#include "infrared.h"

volatile long IF1range[IF_AVR1];
volatile long IF2range[IF_AVR1];
volatile long IF3range[IF_AVR1];
volatile long IF4range[IF_AVR1];

void initIF() {
    /* Initialisation des PINs */
    TRISD&= 0b11111100;        /* OPTOSW1, OPTOSW2 */

    OPTOSW1 = 1;
    OPTOSW2 = 1;

    OpenADC(ADC_FOSC_64         /* A/D clock source (forcément Fosc / 64 car PIC à 48 MHz) */
        & ADC_RIGHT_JUST        /* A/D result justification */
        & ADC_0_TAD,            /* A/D acquisition time select (pas la peine d'attendre avant le */
                                /* début de la conversion car l'acquisition a largement eu le */
                                /* temps de se faire entre chaque interruption, toutes les 1 ms) */
        ADC_INT_OFF             /* A/D Interrupts */
        & ADC_REF_VDD_VSS,      /* A/D voltage configuration */
        ADC_4ANA
    );
}

void interruptIF(void) {
    static char state = 0;
    static char IF1_idx = 0;
    static char IF2_idx = 0;
    static long IF1range_0[IF_AVR0];
    static long IF2range_0[IF_AVR0];
    static long IF3range_0[IF_AVR0];
    static long IF4range_0[IF_AVR0];

    char i;

    switch (state) {
        case 0:
            // Démarre la convertion sur RA0
            SelChanConvADC(ADC_CH0);
            state = 1;
        break;

        case 1:
            // Récupère le résultat de la conversion
            if (!BusyADC()) {
                IF1range_0[IF1_idx] = ReadADC();
                state = 2;
            }
        break;

        case 2:
            // Démarre la convertion sur RA1
            SelChanConvADC(ADC_CH1);
            state = 3;
        break;

        case 3:
            // Récupère le résultat de la conversion
            if (!BusyADC()) {
                IF2range_0[IF1_idx] = ReadADC();
                state = 4;
            }
        break;

        case 4:
            // Démarre la convertion sur RA2
            SelChanConvADC(ADC_CH2);
            state = 5;
        break;

        case 5:
            // Récupère le résultat de la conversion
            if (!BusyADC()) {
                IF3range_0[IF1_idx] = ReadADC();
                state = 6;
            }
        break;

        case 6:
            // Démarre la convertion sur RA3
            SelChanConvADC(ADC_CH3);
            state = 7;
        break;

        case 7:
            // Récupère le résultat de la conversion
            if (!BusyADC()) {
                IF4range_0[IF1_idx] = ReadADC();
                state = 8;
            }
        break;

        case 8:
            // Traitement des résultats
            IF1_idx++;

            if (IF1_idx >= IF_AVR0) {
                IF1_idx = 0;
                IF1range[IF2_idx] = 0;
                IF2range[IF2_idx] = 0;
                IF3range[IF2_idx] = 0;
                IF4range[IF2_idx] = 0;

                for (i = 0; i < IF_AVR0; i++) {
                    IF1range[IF2_idx]+= IF1range_0[i];
                    IF2range[IF2_idx]+= IF2range_0[i];
                    IF3range[IF2_idx]+= IF3range_0[i];
                    IF4range[IF2_idx]+= IF4range_0[i];
                }

                IF2_idx++;
    
                if (IF2_idx >= IF_AVR1)
                    IF2_idx = 0;
            }

            state = 0;
        break;

        default:
            state = 0;
    }
}

long getIFRange(char sensor) {
    char i;
    long IFrange = 0;

    if (sensor == 0) {
        for (i = 0; i < IF_AVR1; i++)
            IFrange+= IF1range[i];
    }
    else if (sensor == 1) {
        for (i = 0; i < IF_AVR1; i++)
            IFrange+= IF2range[i];
    }
    else if (sensor == 2) {
        for (i = 0; i < IF_AVR1; i++)
            IFrange+= IF3range[i];
    }
    else if (sensor == 3) {
        for (i = 0; i < IF_AVR1; i++)
            IFrange+= IF4range[i];
    }
    else
	    return 0; 

    return (long) (((float) IFrange / IF_AVR0 / IF_AVR1) * 5.0 / 1023.0 * 1000.0);
}

#endif
