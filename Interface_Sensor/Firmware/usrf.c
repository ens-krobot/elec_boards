/**
 * @file usrf.c
 * US Rangefinder
*/

#ifndef USRF_C
#define USRF_C

#include "usrf.h"

void usrfMeasure(BYTE id) {    
    OpenI2C(MASTER, SLEW_OFF);
    StartI2C();
    IdleI2C();
    WriteI2C((id << 1) | 0b1);              // 7-bit address + write
    WriteI2C('b');    
    StopI2C();
    IdleI2C();
    CloseI2C();
}

WORD usrfGet(BYTE id) {
    WORD_VAL value;
    UINT count;
    BOOL timeOut;

    OpenI2C(MASTER, SLEW_OFF);
    StartI2C();
    IdleI2C();
    WriteI2C((id << 1) | 0b1);              // 7-bit address + write
    WriteI2C('g');

    RestartI2C();
    IdleI2C();
    WriteI2C(id << 1);      // 7-bit address + read

    SSPCON2bits.RCEN = 1;
    timeOut = TRUE;

    for (count = 0; count < 1000; count++) {
        if (DataRdyI2C()) {
            value.byte.HB = SSPBUF;
            timeOut = FALSE;
            break;
        }
    }

    if (!timeOut) {
        AckI2C();
        IdleI2C();

        SSPCON2bits.RCEN = 1;
        timeOut = TRUE;
    
        for (count = 0; count < 1000; count++) {
            if (DataRdyI2C()) {
                value.byte.LB = SSPBUF;
                timeOut = FALSE;
                break;
            }    
        }
    }

    if (timeOut)
        value.Val = 0;

    NotAckI2C();
    IdleI2C();
    StopI2C();
    IdleI2C();
    CloseI2C();

    return value.Val;
}

#endif
