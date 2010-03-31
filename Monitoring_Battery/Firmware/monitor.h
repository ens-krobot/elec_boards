/**
 * @file monitor.h
*/

#ifndef MONITOR_H
#define MONITOR_H

#include <adc.h>
#include "Compiler.h"
#include "HardwareProfile.h"
#include "GenericTypeDefs.h"

// Paramètres généraux
#define NB_CELLS                   8                ///< Nombre de cellules de batterie installées (normalement 4 pour 12 V et 8 pour 24 V)
#define FULL_CHARGE_THRES       3300                ///< Seuil de charge pleine (en mV)
#define LOW_CHARGE_THRES        2800                ///< Seuil de charge faible (en mV) -- si la tension d'une cellule est inférieure à ce seuil, une alarme est déclenchée
#define ABSENT_CELL_THRES        100                ///< Seuil cellule suspectée de ne pas être connectée
#define CURRENT_LOOP             6.0                ///< Nombre de boucles de courant dans le capteur à effet Hall (forcément un entier)

// Constantes calculées
#define CURRENT_GAIN         ((50.0/0.625)/CURRENT_LOOP)

// Entrées / sorties
#define BUZZER               PORTCbits.RC2

void interruptMonitor(void);
void initMonitor(void);

int getCellVoltage(char cell);
long getCurrent(void);
BOOL getPowerState(void);
BYTE getBatteryState(void);

#endif
