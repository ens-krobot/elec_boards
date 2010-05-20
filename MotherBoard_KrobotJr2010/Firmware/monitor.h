/*
 * Monitor on USART using ChibiOS/RT Shell
 * Xavier Lagorce
 */

#ifndef HEADER__MONITOR
#define HEADER__MONITOR

// Default to USART1
#ifndef MONITOR_USART
#define MONITOR_USART SD1
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "can_monitor.h"
#include "ax12.h"
#include "lift.h"

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
