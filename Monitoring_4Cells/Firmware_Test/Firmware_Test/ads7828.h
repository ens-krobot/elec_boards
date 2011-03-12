/*
 * Wrapper to use the ADS7828 analog to digital converter
 * Olivier Bichler
 */

#ifndef HEADER__ADS7828
#define HEADER__ADS7828

#define ADS7828_VREF    2.5
#define ADS7828_RES    12
#define ADS7828_LSB     (ADS7828_VREF/((1 << ADS7828_RES) - 1.0))

#define ADS7828_ADDR_BASE   0x90
#define ADS7828_ADDR_A0   0x02
#define ADS7828_ADDR_A1   0x04

#define ADS7828_CMD_PD0   0x04
#define ADS7828_CMD_PD1   0x08
#define ADS7828_CMD_C0   0x10
#define ADS7828_CMD_C1   0x20
#define ADS7828_CMD_C2   0x40
#define ADS7828_CMD_SD   0x80

#include <drv/i2c.h>

unsigned int adc7828_measure(I2c* i2c, unsigned char addr, unsigned char ch);

#endif

