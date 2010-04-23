/*
 * CPU Load calculator
 * Xavier Lagorce
 */

#ifndef HEADER__CPULOAD
#define HEADER__CPULOAD

#include "ch.h"
#include "chthreads.h"
#include "chtypes.h"

void CPULoadInit(void);
uint16_t getCPUload(void);
#endif
