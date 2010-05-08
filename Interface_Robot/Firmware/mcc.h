/**
 * @file mcc.h
*/

#ifndef MCC_H
#define MCC_H

#include <pwm.h>
#include <timers.h>     // fonctions pour les timers
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"
#include "PcInterface.h"
#include "error.h"

// Paramètres généraux
#define PWM_FREQ                10000               ///< Fréquence du PWM (en KHz)
#define H_BRIDGE_VOLTAGE         24.0
#define MOTOR1_VOLTAGE           15.0
#define MOTOR2_VOLTAGE           15.0

// Entrées / sorties
#ifndef KROBOT_2010
    #define ENC1_A                PORTAbits.RA4
    #define ENC1_B                PORTAbits.RA5
    #define ENC1_NI               PORTEbits.RE0
    #define M1_EN                 PORTBbits.RB2
    #define M1_PWM                PORTCbits.RC2
    #define M1_SENS               PORTAbits.RA0

    #define ENC2_A                PORTCbits.RC0
    #define ENC2_B                PORTAbits.RA2
    #define ENC2_NI               PORTAbits.RA3
    #define M2_EN                 PORTBbits.RB5
    #define M2_PWM                PORTBbits.RB3
    #define M2_SENS               PORTBbits.RB4
#else
    #define M1_EN                 PORTBbits.RB2
    #define M1_PWM                PORTCbits.RC2
    #define M1_INA                PORTBbits.RB5
    #define M1_INB                PORTBbits.RB4
#endif

#define TOR1                  PORTDbits.RD6
#define TOR2                  PORTDbits.RD7

// Constantes programme
#define MOTOR_RIGHT                 1               ///< Sélection du moteur de droite (moteur 1)
#define MOTOR_LEFT                  2               ///< Sélection du moteur de gauche (moteur 2)
#define MOTOR_BOTH                  3               ///< Sélection des 2 moteurs simultanément

void initMCC(char withEncoder);
void interruptMotor1(void);
void interruptMotor2(void);

void enableMotor(char axis);
void disableMotor(char axis);
void move(char axis, char sens, BYTE speed, unsigned long duration);
BOOL checkTOR(void);

#endif
