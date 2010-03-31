/**
 * @file rangefinder.h
 * G�re la t�l�m�trie.
 *
 * @todo    - Fonctions de calibration
 *          - Moyennage de la distance
*/

#ifndef RANGEFINDER_H
#define RANGEFINDER_H

#include <timers.h>     // fonctions pour les timers
#include <delays.h>
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"
#include "eeprom.h"
#include "../PcInterface.h"

extern volatile char Rangefinder;
extern volatile unsigned char CalibrationData[64];

// Param�tres g�n�raux
#define MAX_PULSE                10                ///< Nombre de p�riodes dans la trame d'�mission                                                [default: 10]
#define PULSE_FREQ            40000                ///< Fr�quence de la trame d'�mission (en Hz)                                                   [default: 40000]
#define SOUND_SPEED             340                ///< Vitesse du son (en m/s)                                                                    [default: 340]
//#define MEAS_OFFSET              20                ///< Offset de la mesure (en mm)                                                                [default: 20]
//#define DELAY_PULSE             250                ///< D�lais avant la prise en compte du signal � la r�ception (en us)                           [default: 250]
#define DELAY_RCPT              100                ///< D�lais avant de passer au t�l�m�tre suivant (en us)                                        [default: 100]
#define RCPT_TIMER_PER          500                ///< P�riode de comptage du temps �coul�, pour la r�ception (en us)                             [default: 500]
#define RCPT_MAX_DELAY        18000                ///< D�lais d'attente maximale � la r�ception (en us)                                           [default: 18000]
//#define START_THRESHOLD          15                ///< Seuil initial pour la comparaison (de 1 � 15, 15 correspondant � 66% de Vref)              [default: 15]

//#define EN_DYN_CALIBRATION

#define START_THRESHOLD        (CalibrationData[8*Rangefinder])
//#define MIN_THRESHOLD          (CalibrationData[8*Rangefinder + 1])
#define DELAY_PULSE            ((((WORD) CalibrationData[8*Rangefinder + 2]) << 8) | CalibrationData[8*Rangefinder + 3])
//#define SOUND_SPEED            ((((WORD) CalibrationData[8*Rangefinder + 4]) << 8) | CalibrationData[8*Rangefinder + 5])
#define MEAS_OFFSET            ((((WORD) CalibrationData[8*Rangefinder + 6]) << 8) | CalibrationData[8*Rangefinder + 7])

/// Si non comment�, active le seuil dynamique.
/// Le seuil dynamique permet d'augmenter la sensibilit� de la d�tection � mesure que le temps - et donc la distance - augmente, ce qui permet
/// d'augmenter la port�e du t�l�m�tre tout en concervant une bon rapport signal � bruit pour les mesures de faibles distances.
#define EN_DYN_THRESHOLD
#define THRESHOLD_STEP            1                ///< Pas de diminution du seuil, tous les RCPT_TIMER_PER (uniquement avec seuil dynamique)
#define MIN_THRESHOLD             1                ///< Seuil minimal, de 1 � 15 (uniquement avec seuil dynamique)

// Constantes calcul�es
#define CYCLE_FREQ            (CLOCK_FREQ / 4.0)

// Entr�es / sorties
#define ADDR0                PORTDbits.RD3        ///< Bit 0 d'adresse du t�l�m�tre
#define ADDR1                PORTDbits.RD2        ///< Bit 1 d'adresse du t�l�m�tre
#define ADDR2                PORTCbits.RC6        ///< Bit 2 d'adresse du t�l�m�tre
#define EM_EN                PORTDbits.RD7        ///< Signal ENABLE du circuit d'�mission
#define RCPT_EN              PORTDbits.RD6        ///< Signal ENABLE du circuit de r�ception
#define PULSE                PORTDbits.RD5        ///< Signal PULSE pour l'envoie de trames

// Constantes du t�l�m�tre
#define MODE_MEASURE              0
#define MODE_CALIBRATE            1
#define MODE_CALIBRATE_AUTO       2

/**
 * Initialise l'interface avec le syst�me de t�l�m�trie.
 * Cette commande ne doit �tre appel�e qu'une seule fois au d�but du programme.
*/
#define initRangefinder() {                                                                                \
    char rf;                                                                                               \
                                                                                                           \
    ADCON1&= 0b11111011;                                                                                   \
    TRISC&= 0b10111111;                                                                                    \
    PORTC&= 0b10111111;                                                                                    \
    TRISD&= 0b00010011;                                                                                    \
    PORTD&= 0b00010011;                                                                                    \
    CMCON = 0b00010110;                                                                                    \
    CVRCON = (0b10110000 | (START_THRESHOLD & 0b1111));                                                    \
                                                                                                           \
    for (rf = 0; rf < 8; rf++) {                                                                           \
        CalibrationData[8*rf] = ReadEEPROM(8*rf + 1);                                                      \
        CalibrationData[8*rf+1] = ReadEEPROM(8*rf + 2);                                                    \
        CalibrationData[8*rf+2] = ReadEEPROM(8*rf + 3);                                                    \
        CalibrationData[8*rf+3] = ReadEEPROM(8*rf + 4);                                                    \
        CalibrationData[8*rf+4] = ReadEEPROM(8*rf + 5);                                                    \
        CalibrationData[8*rf+5] = ReadEEPROM(8*rf + 6);                                                    \
        CalibrationData[8*rf+6] = ReadEEPROM(8*rf + 7);                                                    \
        CalibrationData[8*rf+7] = ReadEEPROM(8*rf + 8);                                                    \
    }                                                                                                      \
}

void interruptRangefinder(void);
void interruptRangefinder2(void);

BOOL measureDistance(char addr);
void selectRangefinder(char addr);
void sendPulse(void);
long getMeasuredDist(char addr);
BYTE calibrate(char addr, BOOL skipMeas);
BYTE calibrateAuto(char addr);
long quickSelect(long arr[], int n);

#endif
