/*
 * Motor controller interface
 * This is a quick implementation based of the stm32lib from ST and from
 * previous implementations.
 *
 * This is supposed to be quick and dirty, waiting for BeRTOS proper
 * PWMs integration, to allow work on other systems using PWMs.
 *
 * Author : Xavier Lagorce
 */

#ifndef HEADER__MOTOR
#define HEADER__MOTOR

#define MOTOR1    1
#define MOTOR2    2
#define MOTOR3    4
#define MOTOR4    8

#define MOTOR_STOP    1
#define MOTOR_BRAKE   2

#define MAX_PWM  3600

#include <drv/gpio_stm32.h>
#include <drv/clock_stm32.h>
#include "hw/hw_led.h"

void motorsInit(void);
void enableMotor(uint8_t motor);
void disableMotor(uint8_t motor);
void motorSetSpeed(uint8_t motor, int32_t speed);
void motorStop(uint8_t motor, uint8_t mode);
void motorSetMaxPWM(uint8_t motor, int32_t maxPWM);

#endif
