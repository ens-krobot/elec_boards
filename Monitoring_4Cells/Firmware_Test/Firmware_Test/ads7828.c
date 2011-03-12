/*
 * Wrapper to use the ADS7828 analog to digital converter
 * Olivier Bichler
 */

#include "ads7828.h"

unsigned int adc7828_measure(I2c* i2c, unsigned char addr, unsigned char ch) {
    unsigned char value[2] = {0};
    unsigned char cmd = ADS7828_CMD_PD0
                         | ADS7828_CMD_PD1
                         | ((ch/2) << 4)
                         | ADS7828_CMD_SD;

    if (ch % 2)
        cmd|= ADS7828_CMD_C2;

    i2c_start_w(i2c, addr, 1, I2C_STOP);
    i2c_write(i2c, &cmd, 1);
    i2c_start_r(i2c, addr, 2, I2C_STOP);
    i2c_read(i2c, &value, 2);

    return ((value[0] << 8) | value[1]);
}

