/*
 * Wrapper to use the Encoder interface
 * Xavier Lagorce
 */

#ifndef HEADER__ENCODER
#define HEADER__ENCODER

#define ENCODER1   0
#define ENCODER2   1
#define ENCODER3   2
#define ENCODER4   3

#define ENCODER_DIR_UP     0
#define ENCODER_DIR_DOWN   1

#include <drv/gpio_stm32.h>
#include <drv/clock_stm32.h>

/*
 * Intializes the encoder interface
 */
void encodersInit(void);
/*
 * Sets the scaling factors for readout helper
 */
void setEncoderScaling(uint8_t encoder, uint16_t origin, float scale);

/*
 * Returns the current position of a particular encoder
 */
uint16_t getEncoderPosition(uint8_t encoder);
/*
 * Gets the corrent value of an encoder after scaling process
 */
float getEncoderPosition_f(uint8_t encoder);

/*
 * Resets the counter register associated to a given encoder
 */
void resetEncoderPosition(uint8_t encoder);

/*
 * Gets the direction of the evolution of an encoder value
 */
uint8_t getEncoderDirection(uint8_t encoder);

#endif
