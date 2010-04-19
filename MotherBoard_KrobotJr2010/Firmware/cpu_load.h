/*
 * CPU Load calculator
 * Xavier Lagorce
 */

#ifndef HEADER__CPULOAD
#define HEADER__CPULOAD

#include "ch.h"
#include "vt.h"
#include "chtypes.h"

extern volatile uint16_t CPUload;

void CPULoadInit(void);

#endif
