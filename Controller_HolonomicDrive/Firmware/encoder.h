/*
 * Wrapper to use the Encoder interface
 * Xavier Lagorce
 */

#ifndef HEADER__ENCODER
#define HEADER__ENCODER

#define ENCODER1   1
#define ENCODER2   2
#define ENCODER3   3

#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

void encodersInit(void);
uint16_t getEncoderPosition(uint8_t encoder);
void resetEncoderPosition(uint8_t encoder);

#endif
