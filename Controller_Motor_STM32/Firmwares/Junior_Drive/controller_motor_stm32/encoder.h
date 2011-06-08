#ifndef HEADER__ENCODER
#define HEADER__ENCODER

#include <drv/gpio_stm32.h>
#include <drv/clock_stm32.h>
#include "hw/hw_led.h"

#define ENCODER1 1 //encoder connected to the motor 1
#define ENCODER2 2// ...
#define ENCODER3 4
#define ENCODER4 8

void encodersInit(void);

uint16_t getEncoderCount(unsigned short encoder);

void resetEncoderCount(unsigned short encoders);

uint8_t getEncoderDirection(unsigned short encoder);

#endif
