/**
 * @file error.c
*/

#ifndef ERROR_C
#define ERROR_C

#include "error.h"

volatile BYTE err[32] = {0};
volatile BYTE errno = 0;

void error(BYTE _err) {
    if (err[errno] > 0)
        errno = (errno + 1) % 32;

    err[errno] = _err;
}

#endif
