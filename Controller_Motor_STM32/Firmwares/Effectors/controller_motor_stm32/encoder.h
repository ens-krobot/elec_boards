/*
 * Wrapper to use the Encoder interface
 * Xavier Lagorce
 */

#ifndef HEADER__ENCODER
#define HEADER__ENCODER

#define ENCODER1   1
#define ENCODER2   2
#define ENCODER3   3
#define ENCODER4   4

#define ENCODER_DIR_UP     0
#define ENCODER_DIR_DOWN   1

#include <drv/gpio_stm32.h>
#include <drv/clock_stm32.h>

void encodersInit(void);
uint16_t getEncoderPosition(uint8_t encoder);
void resetEncoderPosition(uint8_t encoder);
uint8_t getEncoderDirection(uint8_t encoder);

#endif
