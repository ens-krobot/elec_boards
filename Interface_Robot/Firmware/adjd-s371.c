/**
 * @file mcc.c
*/

#ifndef ADJD_C
#define ADJD_C

#include "adjd-s371.h"

void initAdjd(void) {
    adjdWriteRegister(CAP_RED, 0x05);
    adjdWriteRegister(CAP_GREEN, 0x05);
    adjdWriteRegister(CAP_BLUE, 0x05);
    adjdWriteRegister(CAP_CLEAR, 0x05);

    adjdWriteRegister(INT_RED_LO, 0xC4);
    adjdWriteRegister(INT_RED_HI, 0x09);
    adjdWriteRegister(INT_GREEN_LO, 0xC4);
    adjdWriteRegister(INT_GREEN_HI, 0x09);
    adjdWriteRegister(INT_BLUE_LO, 0xC4);
    adjdWriteRegister(INT_BLUE_HI, 0x09);
    adjdWriteRegister(INT_CLEAR_LO, 0xC4);
    adjdWriteRegister(INT_CLEAR_HI, 0x09);
}

void adjdWriteRegister(BYTE regName, BYTE regValue) {
    OpenI2C(MASTER, SLEW_OFF);

//    EEAckPolling(ADJD_WRITE);

//    EEByteWrite(ADJD_WRITE, regName, regValue);

    StartI2C();
    IdleI2C();        // Absolument nécessaire
    WriteI2C(ADJD_WRITE);
    WriteI2C(regName);         // Register name
    WriteI2C(regValue);        // Register value
    StopI2C();
    CloseI2C();
}

BYTE adjdReadRegister(BYTE regName) {
    UINT count;
    BYTE regValue = 0;

    OpenI2C(MASTER, SLEW_OFF);

//    EEAckPolling(ADJD_WRITE);

//    regValue = EERandomRead(ADJD_WRITE, regName);

    StartI2C();
    IdleI2C();        // Absolument nécessaire
    WriteI2C(ADJD_WRITE);
    WriteI2C(regName);        // Register name

    RestartI2C();
    IdleI2C();        // Absolument nécessaire
    WriteI2C(ADJD_READ);

    SSPCON2bits.RCEN = 1;                    // start receiving

    for (count = 0; count < 1000; count++) {
        if (DataRdyI2C()) {
            regValue = SSPBUF;
            break;
        }
    }

    NotAckI2C();
    IdleI2C();
    StopI2C();
    IdleI2C();
    CloseI2C();

    return regValue;
}

#endif
