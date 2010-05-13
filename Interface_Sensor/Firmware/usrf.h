/**
 * @file usrf.h
 * US Rangefinder
*/

#ifndef USRF_H
#define USRF_H

#include <i2c.h>
#include <delays.h>
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"

void usrfMeasure(BYTE id);
WORD usrfGet(BYTE id);

#endif
