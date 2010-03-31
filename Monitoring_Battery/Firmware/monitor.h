/**
 * @file monitor.h
*/

#ifndef MONITOR_H
#define MONITOR_H

#include <adc.h>
#include "Compiler.h"
#include "HardwareProfile.h"
#include "GenericTypeDefs.h"

// Param�tres g�n�raux
#define NB_CELLS                   8                ///< Nombre de cellules de batterie install�es (normalement 4 pour 12 V et 8 pour 24 V)
#define FULL_CHARGE_THRES       3300                ///< Seuil de charge pleine (en mV)
#define LOW_CHARGE_THRES        2800                ///< Seuil de charge faible (en mV) -- si la tension d'une cellule est inf�rieure � ce seuil, une alarme est d�clench�e
#define ABSENT_CELL_THRES        100                ///< Seuil cellule suspect�e de ne pas �tre connect�e
#define CURRENT_LOOP             6.0                ///< Nombre de boucles de courant dans le capteur � effet Hall (forc�ment un entier)

// Constantes calcul�es
#define CURRENT_GAIN         ((50.0/0.625)/CURRENT_LOOP)

// Entr�es / sorties
#define BUZZER               PORTCbits.RC2

void interruptMonitor(void);
void initMonitor(void);

int getCellVoltage(char cell);
long getCurrent(void);
BOOL getPowerState(void);
BYTE getBatteryState(void);

#endif
