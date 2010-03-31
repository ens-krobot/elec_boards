/**
 * @file mcc.h
*/

#ifndef ADJD_H
#define ADJD_H

#include <i2c.h>
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"
#include "PcInterface.h"
#include "error.h"

#define ADJD_WRITE      0xE8        // Default ADJD-S371 I2C address - write
#define ADJD_READ       0xE9        // Default ADJD-S371 I2C address - read

// FN
#define CTRL            0x00
#define CONFIG          0x01

// SENSOR
#define CAP_RED         0x06
#define CAP_GREEN       0x07
#define CAP_BLUE        0x08
#define CAP_CLEAR       0x09
#define INT_RED_LO      0x0A
#define INT_RED_HI      0x0B
#define INT_GREEN_LO    0x0C
#define INT_GREEN_HI    0x0D
#define INT_BLUE_LO     0x0E
#define INT_BLUE_HI     0x0F
#define INT_CLEAR_LO    0x10
#define INT_CLEAR_HI    0x11

// SAMPLE DATA
#define DATA_RED_LO     0x40
#define DATA_RED_HI     0x41
#define DATA_GREEN_LO   0x42
#define DATA_GREEN_HI   0x43
#define DATA_BLUE_LO    0x44
#define DATA_BLUE_HI    0x45
#define DATA_CLEAR_LO   0x46
#define DATA_CLEAR_HI   0x47

// OFFSET DATA
#define OFFSET_RED      0x48
#define OFFSET_GREEN    0x49
#define OFFSET_BLUE     0x4A
#define OFFSET_CLEAR    0x4B

void initAdjd(void);
void adjdWriteRegister(BYTE regName, BYTE regValue);
BYTE adjdReadRegister(BYTE regName);

#endif
