/*
 * CAN monitor to receive/send informations
 * Xavier Lagorce
 */

#ifndef HEADER__CANMONITOR
#define HEADER__CANMONITOR

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "ch.h"
#include "hal.h"
#include "can.h"
#include "encoder.h"
#include "speed_control.h"
#include "trajectory.h"

extern Thread *canrtp;
extern Thread *canttp;

// Data structures to group values in few bytes
struct speeds_t {
  signed motor1 : 17  __attribute__((__packed__));
  signed motor2 : 17  __attribute__((__packed__));
  signed motor3 : 17  __attribute__((__packed__));
} ;

struct move_t {
  signed ptX   : 12  __attribute__((__packed__));
  signed ptY   : 12  __attribute__((__packed__));
  signed vX    : 12  __attribute__((__packed__));
  signed vY    : 12  __attribute__((__packed__));
  signed omega : 12  __attribute__((__packed__));
} ;

// Data structures to express messages
typedef union {
  struct speeds_t speeds;
  uint8_t  data[8];
} speedMsg_t;

typedef union {
  struct move_t  move;
  uint8_t data[8];
} moveMsg_t;

// Prototypes
void canMonitorInit(void);
void canMonitorStart(void);

#endif
