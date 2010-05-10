/**
 * @file servo.c
*/

#ifndef SERVO_C
#define SERVO_C

#include "servo.h"

volatile char glbServoAngle[5] = {0};
volatile BYTE glbServoEnabled = 0;

void initServos() {
    /* Initialisation des PINs */
    TRISD&= 0b11100011;        /* SERVO1, SERVO2, SERVO3 */

    #ifndef KROBOT_2010
        TRISA&= 0b11111101;        /* SERVO4 */
    #endif

    TRISC&= 0b11111101;        /* SERVO5 */

    /* Configuration du timer3 */
    PIE2bits.TMR3IE = 1;        /* Autorise les interruption par dépassement du Timer (Timer overflow) */
    OpenTimer3(TIMER_INT_ON     /* active le timer3 */
         & T3_16BIT_RW          /* compte sur 16 bits */
         & T3_SOURCE_INT        /* utilise l'horloge interne */
         & T3_PS_1_8            /* incrémente le compteur à chaque cycle (1:1) */
         & T3_OSC1EN_OFF        /* pas d'oscillateur sur le timer3 */
         & T3_SYNC_EXT_OFF      /* ne pas se synchroniser sur une horloge externe */
    );
    WriteTimer3(0);

    /* Initialisation de l'état des PINs */
    SERVO1 = 0;
    SERVO2 = 0;
    SERVO3 = 0;

    #ifndef KROBOT_2010
        SERVO4 = 0;
    #endif

    SERVO5 = 0;
}

void interruptServo(void) {
    static BYTE servo = 0;
    static unsigned int elapsedTime = 0;

    SERVO1 = 0;
    SERVO2 = 0;
    SERVO3 = 0;

    #ifndef KROBOT_2010
        SERVO4 = 0;
    #endif

    SERVO5 = 0;

servo0:
    while (servo < 5 && !(glbServoEnabled & (0b1 << servo)))
        servo++;

    if (servo < 5) {
        switch (servo) {
            case 0:     SERVO1 = 1;    break;
            case 1:     SERVO2 = 1;    break;
            case 2:     SERVO3 = 1;    break;
            case 3:
                #ifndef KROBOT_2010
                    SERVO4 = 1;
                #endif
                break;
            case 4:     SERVO5 = 1;    break;
        }

        elapsedTime+= SERVO_ORIGIN_POS + SERVO_PCM_STEP * ((unsigned int) glbServoAngle[servo]);
        WriteTimer3(65535 - (unsigned int) (CYCLE_FREQ / 1e6 * (SERVO_ORIGIN_POS + SERVO_PCM_STEP * (float) glbServoAngle[servo])));
        servo++;
    }
    else {
        servo = 0;

        if (elapsedTime < SERVO_PULSE_PER) {
            WriteTimer3(65535 - (unsigned int) (CYCLE_FREQ / 1e6 * (SERVO_PULSE_PER - (float) elapsedTime)));
            elapsedTime = 0;
        }
        else {
            elapsedTime = 0;
            goto servo0;
        }
    }
}

/**
 * Commande l'angle d'un servomoteur
 *
 * @param        servo        numéro du servomoteur à partir de 0 (de 0 à 4)
 * @param        angle        angle de consigne à maintenir, en degré
*/
void setServoAngle(BYTE servo, char angle) {
    glbServoAngle[servo] = angle;
}

/**
 * Active un servomoteur
 * Seul les servomoteurs qui auront été activés au préalable seront commandés.
 *
 * @param        servo        numéro du servomoteur à activer à partir de 0 (de 0 à 4)
*/
void enableServo(BYTE servo) {
    glbServoEnabled|= (0b1 << servo);
}

/**
 * Désactive un servomoteur
 *
 * @param        servo        numéro du servomoteur à désactiver à partir de 0 (de 0 à 4)
*/
void disableServo(BYTE servo) {
    glbServoEnabled&= ~(0b1 << servo);
}

#endif
