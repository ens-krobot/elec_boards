#ifndef INFRARED_H
#define INFRARED_H

#include <adc.h>
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"
#include "PcInterface.h"
#include "error.h"

// Paramètres généraux
#define IF_AVR0             1
#define IF_AVR1             1

// Entrées / sorties
#define OPTOSW1                    PORTDbits.RD0
#define OPTOSW2                    PORTDbits.RD1

void initIF(void);
void interruptIF(void);
long getIFRange(char sensor);

#endif
