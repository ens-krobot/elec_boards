#ifndef INFRARED_H
#define INFRARED_H

#include <adc.h>
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"
#include "PcInterface.h"
#include "error.h"

// Param�tres g�n�raux
#define IF_AVR0             1
#define IF_AVR1             1

// Entr�es / sorties
#define OPTOSW1                    PORTDbits.RD0
#define OPTOSW2                    PORTDbits.RD1

void initIF(void);
void interruptIF(void);
long getIFRange(char sensor);

#endif
