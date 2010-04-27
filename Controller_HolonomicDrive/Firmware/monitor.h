/*
 * Monitor on USART using ChibiOS/RT Shell
 * Xavier Lagorce
 */

#ifndef HEADER__MONITOR
#define HEADER__MONITOR

// Default to USART2
#ifndef MONITOR_USART
#define MONITOR_USART SD2
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "encoder.h"
#include "motor.h"
#include "cpu_load.h"
#include "speed_control.h"
#include "trajectory.h"

extern Thread *cdtp;

/*
 * Macros
 */
#define cputs(msg) chMsgSend(cdtp, (msg_t)msg) 

/*
 * Prototypes
 */
void monitorInit(void);

#endif
