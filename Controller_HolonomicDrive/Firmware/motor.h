/*
 * Motor controller interface
 * Xavier Lagorce
 */

#ifndef HEADER__MOTOR
#define HEADER__MOTOR

#define MOTOR1    1
#define MOTOR2    2
#define MOTOR3    4

#define MOTOR_STOP    1
#define MOTOR_BRAKE   2

#define MAX_PWM  3600

#include "ch.h"
#include "nvic.h"
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

void motorsInit(void);
void enableMotor(uint8_t motor);
void disableMotor(uint8_t motor);
void motorSetSpeed(uint8_t motor, int32_t speed);
void motorStop(uint8_t motor, uint8_t mode);

#endif
