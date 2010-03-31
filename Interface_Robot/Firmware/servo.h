/**
 * @file servo.h
*/

#ifndef SERVO_H
#define SERVO_H

#include <timers.h>     // fonctions pour les timers
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"
#include "PcInterface.h"
#include "error.h"

// Paramètres généraux
#define SERVO_ORIGIN_POS        1500            ///< Durée de l'impulsion pour l'angle 0° (milieu) (en us)
#define SERVO_PCM_STEP            10            ///< Pas (durée = angle de 1°) (en us)
#define SERVO_PULSE_PER        20000            ///< Période des impulsions (en us)

// Constantes calculées
#define CYCLE_FREQ                (CLOCK_FREQ / 32.0)

// Entrées / sorties
#define SERVO1                    PORTDbits.RD2
#define SERVO2                    PORTDbits.RD3
#define SERVO3                    PORTDbits.RD4
#define SERVO4                    PORTAbits.RA1
#define SERVO5                    PORTCbits.RC1

#define initServos() {                                                                                             \
    /* Initialisation des PINs */                                                                                  \
    TRISD&= 0b11100011;        /* SERVO1, SERVO2, SERVO3 */                                                        \
    TRISA&= 0b11111101;        /* SERVO4 */                                                                        \
    TRISC&= 0b11111101;        /* SERVO5 */                                                                        \
                                                                                                                   \
    /* Configuration du timer3 */                                                                                  \
    PIE2bits.TMR3IE = 1;        /* Autorise les interruption par dépassement du Timer (Timer overflow) */          \
    OpenTimer3(TIMER_INT_ON     /* active le timer3 */                                                             \
         & T3_16BIT_RW          /* compte sur 16 bits */                                                           \
         & T3_SOURCE_INT        /* utilise l'horloge interne */                                                    \
         & T3_PS_1_8            /* incrémente le compteur à chaque cycle (1:1) */                                  \
         & T3_OSC1EN_OFF        /* pas d'oscillateur sur le timer3 */                                              \
         & T3_SYNC_EXT_OFF      /* ne pas se synchroniser sur une horloge externe */                               \
    );                                                                                                             \
    WriteTimer3(0);                                                                                                \
                                                                                                                   \
    /* Initialisation de l'état des PINs */                                                                        \
    SERVO1 = 0;                                                                                                    \
    SERVO2 = 0;                                                                                                    \
    SERVO3 = 0;                                                                                                    \
    SERVO4 = 0;                                                                                                    \
    SERVO5 = 0;                                                                                                    \
}

void interruptServo(void);
void setServoAngle(BYTE servo, char angle);
void enableServo(BYTE servo);
void disableServo(BYTE servo);

#endif
