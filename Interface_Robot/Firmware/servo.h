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

#ifndef KROBOT_2010
    #define SERVO4                    PORTAbits.RA1
#endif

#define SERVO5                    PORTCbits.RC1

void initServos(void);
void interruptServo(void);
void setServoAngle(BYTE servo, char angle);
void enableServo(BYTE servo);
void disableServo(BYTE servo);

#endif
