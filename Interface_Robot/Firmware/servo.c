/**
 * @file servo.c
*/

#ifndef SERVO_C
#define SERVO_C

#include "servo.h"

volatile char glbServoAngle[5] = {0};
volatile BYTE glbServoEnabled = 0;

void interruptServo(void) {
    static BYTE servo = 0;
    static unsigned int elapsedTime = 0;

    SERVO1 = 0;
    SERVO2 = 0;
    SERVO3 = 0;
    SERVO4 = 0;
    SERVO5 = 0;

servo0:
    while (servo < 5 && !(glbServoEnabled & (0b1 << servo)))
        servo++;

    if (servo < 5) {
        switch (servo) {
            case 0:     SERVO1 = 1;    break;
            case 1:     SERVO2 = 1;    break;
            case 2:     SERVO3 = 1;    break;
            case 3:     SERVO4 = 1;    break;
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
    glbServoEnabled^= (0b1 << servo);
}

#endif
