/**
 * @file eeprom.h
 * Fonctions de lecture et d'écriture avec l'EEPROM du PIC.
*/

#ifndef EEPROM_H
#define EEPROM_H

#include <delays.h>
#include "GenericTypeDefs.h"
#include "Compiler.h"

BYTE ReadEEPROM(BYTE addr);
void WriteEEPROM(BYTE addr, BYTE data);

#endif
