/**
 * @file motor.c
 * Gère l'interface avec la carte de puissance du moteur.
*/

#ifndef MOTOR_C
#define MOTOR_C

#include "motor.h"

volatile long Isens1[ISENS_AVR1];
volatile long Isens2[ISENS_AVR1];

/**
 * Fonction d'interruption pour les moteurs.
 * Cette fonction ne doit être appelée que dans une interruption.
 * Elle permet la mesure du courant dans le moteur.
*/
void interruptMotor(void) {
    static char state = 0;
    static char Isens_idx0 = 0;
    static char Isens_idx1 = 0;
    static long Isens1_0[ISENS_AVR0];
    static long Isens2_0[ISENS_AVR0];

    char i;

    switch (state) {
        case 0:
            // Démarre la convertion sur RA0 (Isens1)
            SelChanConvADC(ADC_CH0);
            state = 1;
        break;

        case 1:
            // Récupère le résultat de la conversion
            if (!BusyADC()) {
                Isens1_0[Isens_idx0] = ReadADC();
                state = 2;
            }
        break;

        case 2:
            // Démarre la convertion sur RA3 (Isens2)
            #if defined(REV_1_0)
                // Board revision 1.0
                SelChanConvADC(ADC_CH3);
            #elif defined(REV_1_1)
                // Board revision 1.1
                SelChanConvADC(ADC_CH1);
            #else
                #error Unknown board revision
            #endif

            state = 3;
        break;

        case 3:
            // Récupère le résultat de la conversion
            if (!BusyADC()) {
                Isens2_0[Isens_idx0] = ReadADC();
                state = 4;
            }
        break;

        case 4:
            // Traitement des résultats
            Isens_idx0++;

            if (Isens_idx0 >= ISENS_AVR0) {
                Isens_idx0 = 0;
                Isens1[Isens_idx1] = 0;
                Isens2[Isens_idx1] = 0;

                for (i = 0; i < ISENS_AVR0; i++) {
                    Isens1[Isens_idx1]+= Isens1_0[i];
                    Isens2[Isens_idx1]+= Isens2_0[i];
                }

                Isens_idx1++;
    
                if (Isens_idx1 >= ISENS_AVR1)
                    Isens_idx1 = 0;
            }

            state = 0;
        break;

        default:
            state = 0;
    }
}

/**
 * Récupère la mesure du courant dans un moteur.
 * Cette fonction donne une valeur "instantanée moyenne" du courant dans le moteur.
 * En fait, la valeur renvoyée est déjà une moyenne sur un certain nombre d'échantillons.
 *
 * @param        axis        l'axe moteur dont on veut le courant, peut valoir : @n
 *                             #MOTOR_RIGHT      le moteur de droite @n
 *                             #MOTOR_LEFT        le moteur de gauche
 *
 * @return        Isens        courant mesuré, en mA (entier sur 16 bits)
*/
long getIsens(char axis) {
    char i;
    long Isens = 0;

    if (axis == MOTOR_RIGHT) {
        for (i = 0; i < ISENS_AVR1; i++)
            Isens+= Isens1[i];
    }
    else if (axis == MOTOR_LEFT) {
        for (i = 0; i < ISENS_AVR1; i++)
            Isens+= Isens2[i];
    }
    else {
        error(ERR_INVALID_AXIS);
		return 0;
    }    

    return (long) ((((float) Isens / ISENS_AVR0 / ISENS_AVR1) * 5.0 / 1023.0 - 2.5) * ISENS_COEF);
}

/**
 * Active un ou plusieurs axe(s) moteur(s).
 * Cette fonction permet d'activer un axe moteur en vue d'une commande par le LM629.
 * L'activation d'un moteur rend la carte de puissance correspondante réceptive au
 * signal PWM délivré par le LM629.
 *
 * @param        axis        l'axe moteur à activer, peut valoir : @n
 *                             #MOTOR_RIGHT      le moteur de droite uniquement @n
 *                             #MOTOR_LEFT        le moteur de gauche uniquement @n
 *                             #MOTOR_BOTH        les 2 moteurs
*/
void enableMotor(char axis) {
    if (axis & MOTOR_RIGHT)
        EN_MOTOR1 = 1;

    if (axis & MOTOR_LEFT)
        EN_MOTOR2 = 1;
}

/**
 * Désactive un ou plusieurs axe(s) moteur(s).
 * Cette fonction désactive un axe moteur et le rend inerte à toute commande du LM629.
 * Désactiver un moteur revient à désactiver la carte de puissance associée.
 *
 * @param        axis        l'axe moteur à activer, peut valoir : @n
 *                             #MOTOR_RIGHT       le moteur de droite uniquement @n
 *                             #MOTOR_LEFT        le moteur de gauche uniquement @n
 *                             #MOTOR_BOTH        les 2 moteurs
*/
void disableMotor(char axis) {
    if (axis & MOTOR_RIGHT)
        EN_MOTOR1 = 0;

    if (axis & MOTOR_LEFT)
        EN_MOTOR2 = 0;
}

#endif
