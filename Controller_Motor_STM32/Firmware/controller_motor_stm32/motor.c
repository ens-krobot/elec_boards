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

#include "motor.h"
#include <cfg/macros.h>
#include <drv/gpio_stm32.h>
#include "stm32lib/stm32f10x_tim.h"

uint8_t enabledMotors = 0, indMotors = 0;
signed char currentSpeedSign[] = {0, 0, 0, 0};
int32_t maxPWMs[] = {MAX_PWM, MAX_PWM, MAX_PWM, MAX_PWM};
TIM_OCInitTypeDef  TIM_OCInitStructure;

static int32_t staSpeed(int32_t speed, int32_t maxSpeed, uint8_t *ind) {
  if (speed >= maxSpeed) {
    *ind = 1;
    return MAX_PWM;
  } else if (speed <= -maxSpeed) {
    *ind = 1;
    return -maxSpeed;
  } else {
    *ind = 0;
    return speed;
  }
}

/*
 * Initialises TIM2 for PWM generation and associated GPIOs
 */
void motorsInit(void) {

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

  //Enable GPIOA, GPIOB, GPIOC and GPIOD clock
  RCC->APB2ENR |= RCC_APB2_GPIOA | RCC_APB2_GPIOB | RCC_APB2_GPIOC | RCC_APB2_GPIOD;

  //Enable timer clock
  RCC->APB1ENR |= RCC_APB1_TIM2;

  //Setup timer for quadrature encoder interface
  //Motor1 : EN   PA5
  //         INA  PC4
  //         INB  PC5
  //         PWM  PA0
  //         IND  LED1
  //Motor2 : EN   PB15
  //         INA  PB1
  //         INB  PB14
  //         PWM  PA1
  //         IND  LED2
  //Motor3 : EN   PA10
  //         INA  PC10
  //         INB  PC11
  //         PWM  PA2
  //         IND  LED3
  //Motor4 : EN   PD2
  //         INA  PB5
  //         INB  PD9
  //         PWM  PA3
  //         IND  LED4
  stm32_gpioPinConfig(((struct stm32_gpio *)GPIOA_BASE),
                      BV(5) | BV(10),
                      GPIO_MODE_OUT_PP, GPIO_SPEED_50MHZ);
  stm32_gpioPinConfig(((struct stm32_gpio *)GPIOA_BASE),
                      BV(0) | BV(1) | BV(2) | BV(3),
                      GPIO_MODE_AF_PP, GPIO_SPEED_50MHZ);
  stm32_gpioPinConfig(((struct stm32_gpio *)GPIOB_BASE),
                      BV(1) | BV(5) | BV(9) | BV(14) | BV(15),
                      GPIO_MODE_OUT_PP, GPIO_SPEED_50MHZ);
  stm32_gpioPinConfig(((struct stm32_gpio *)GPIOC_BASE),
                      BV(4) | BV(5) | BV(10) | BV(11),
                      GPIO_MODE_OUT_PP, GPIO_SPEED_50MHZ);
  stm32_gpioPinConfig(((struct stm32_gpio *)GPIOD_BASE),
                      BV(2),
                      GPIO_MODE_OUT_PP, GPIO_SPEED_50MHZ);

  // Default value of H-Bridge configuration
  stm32_gpioPinWrite(((struct stm32_gpio *)GPIOA_BASE),
                     BV(5) | BV(10), 0);
  stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE),
                     BV(1) | BV(5) | BV(9) | BV(14) | BV(15), 0);
  stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE),
                     BV(4) | BV(5) | BV(10) | BV(11), 0);
  stm32_gpioPinWrite(((struct stm32_gpio *)GPIOD_BASE),
                     BV(2), 0);

  // TimeBase configuration
  TIM_TimeBaseStructure.TIM_Prescaler     = (uint16_t) (72000000 / 72000000) - 1;;
  TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period        = 3600; // 20 kHz
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  // PWM1 Mode configuration: Channel1
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM2, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
  TIM_OC2Init(TIM2, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
  TIM_OC3Init(TIM2, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
  TIM_OC4Init(TIM2, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

  // All motors are disabled
  enabledMotors = 0;

  //Enable timer Peripherals
  TIM_Cmd(TIM2,ENABLE);

  // Initialize indicators
  LEDS_INIT();
}

/*
 * Enable a motor driver
 */
void enableMotor(uint8_t motor) {

  if (motor & MOTOR1) {
    enabledMotors |= MOTOR1;
    stm32_gpioPinWrite(((struct stm32_gpio *)GPIOA_BASE), BV(5), 1);
  }
  if (motor & MOTOR2) {
    enabledMotors |= MOTOR2;
    stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(15), 1);
  }
  if (motor & MOTOR3) {
    enabledMotors |= MOTOR3;
    stm32_gpioPinWrite(((struct stm32_gpio *)GPIOA_BASE), BV(10), 1);
  }
  if (motor & MOTOR4) {
    enabledMotors |= MOTOR4;
    stm32_gpioPinWrite(((struct stm32_gpio *)GPIOD_BASE), BV(2), 1);
  }
}

/*
 * Disable a motor driver
 */
void disableMotor(uint8_t motor) {

  if (motor & MOTOR1) {
    enabledMotors &= ~MOTOR1;
    stm32_gpioPinWrite(((struct stm32_gpio *)GPIOA_BASE), BV(5), 0);
  }
  if (motor & MOTOR2) {
    enabledMotors &= ~MOTOR2;
    stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(15), 0);
  }
  if (motor & MOTOR3) {
    enabledMotors &= ~MOTOR3;
    stm32_gpioPinWrite(((struct stm32_gpio *)GPIOA_BASE), BV(10), 0);
  }
  if (motor & MOTOR4) {
    enabledMotors &= ~MOTOR4;
    stm32_gpioPinWrite(((struct stm32_gpio *)GPIOD_BASE), BV(2), 0);
  }
}

/*
 * Step one motor's speed
 */
void motorSetSpeed(uint8_t motor, int32_t speed) {

  uint8_t ind = 0;
  int32_t app_speed;

  if (motor & MOTOR1) {
    app_speed = staSpeed(speed, maxPWMs[0], &ind);
    if(app_speed >= 0) {
      if (currentSpeedSign[0] != 1) {
        currentSpeedSign[0] = 1;
        stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(4), 1);
        stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(5), 0);
      }
      TIM_OCInitStructure.TIM_Pulse = (uint16_t)app_speed;
    } else {
      if (currentSpeedSign[0] != -1) {
        currentSpeedSign[0] = -1;
        stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(4), 0);
        stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(5), 1);
      }
      TIM_OCInitStructure.TIM_Pulse = (uint16_t)(-app_speed);
    }
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    if (ind) {
      if ((indMotors & MOTOR1) == 0) {
        LED1_ON();
        indMotors |= MOTOR1;
      }
    } else if ((indMotors & MOTOR1) != 0) {
      LED1_OFF();
      indMotors &= ~MOTOR1;
    }
  }
  if (motor & MOTOR2) {
    app_speed = staSpeed(speed, maxPWMs[1], &ind);
    if(app_speed >= 0) {
      if (currentSpeedSign[1] != 1) {
        currentSpeedSign[1] = 1;
        stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(1), 1);
        stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(14), 0);
      }
      TIM_OCInitStructure.TIM_Pulse = (uint16_t)app_speed;
    } else {
      if (currentSpeedSign[1] != -1) {
        currentSpeedSign[1] = -1;
        stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(1), 0);
        stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(14), 1);
      }
      TIM_OCInitStructure.TIM_Pulse = (uint16_t)(-app_speed);
    }
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    if (ind) {
      if ((indMotors & MOTOR2) == 0) {
        LED2_ON();
        indMotors |= MOTOR2;
      }
    } else if ((indMotors & MOTOR2) != 0) {
      LED2_OFF();
      indMotors &= ~MOTOR2;
    }
  }
  if (motor & MOTOR3) {
    app_speed = staSpeed(speed, maxPWMs[2], &ind);
    if(app_speed >= 0) {
      if (currentSpeedSign[2] != 1) {
        currentSpeedSign[2] = 1;
        stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(10), 1);
        stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(11), 0);
      }
      TIM_OCInitStructure.TIM_Pulse = (uint16_t)app_speed;
    } else {
      if (currentSpeedSign[2] != -1) {
        currentSpeedSign[2] = -1;
        stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(10), 0);
        stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(11), 1);
      }
      TIM_OCInitStructure.TIM_Pulse = (uint16_t)(-app_speed);
    }
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    if (ind) {
      if ((indMotors & MOTOR3) == 0) {
        LED3_ON();
        indMotors |= MOTOR3;
      }
    } else if ((indMotors & MOTOR3) != 0) {
      LED3_OFF();
      indMotors &= ~MOTOR3;
    }
  }
  if (motor & MOTOR4) {
    app_speed = staSpeed(speed, maxPWMs[0], &ind);
    if(app_speed >= 0) {
      if (currentSpeedSign[3] != 1) {
        currentSpeedSign[3] = 1;
        stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(5), 1);
        stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(9), 0);
      }
      TIM_OCInitStructure.TIM_Pulse = (uint16_t)app_speed;
    } else {
      if (currentSpeedSign[3] != -1) {
        currentSpeedSign[3] = -1;
        stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(5), 0);
        stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(9), 1);
      }
      TIM_OCInitStructure.TIM_Pulse = (uint16_t)(-app_speed);
    }
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    if (ind) {
      if ((indMotors & MOTOR4) == 0) {
        LED4_ON();
        indMotors |= MOTOR4;
      }
    } else if ((indMotors & MOTOR4) != 0) {
      LED4_OFF();
      indMotors &= ~MOTOR4;
    }
  }
}

/*
 * Stop a motor using the 'mode' mode.
 */
void motorStop(uint8_t motor, uint8_t mode) {

  if (mode == MOTOR_STOP) {
    if (motor & MOTOR1) {
      stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(4), 0);
      stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(5), 0);
      currentSpeedSign[0] = 0;
    }
    if (motor & MOTOR2) {
      stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(1), 0);
      stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(14), 0);
      currentSpeedSign[1] = 0;
    }
    if (motor & MOTOR3) {
      stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(10), 0);
      stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(11), 0);
      currentSpeedSign[2] = 0;
    }
    if (motor & MOTOR4) {
      stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(5), 0);
      stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(9), 0);
      currentSpeedSign[3] = 0;
    }
  } else if (mode == MOTOR_BRAKE) {
    if (motor & MOTOR1) {
      stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(4), 0);
      stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(5), 0);
      currentSpeedSign[0] = 0;
    }
    if (motor & MOTOR2) {
      stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(1), 0);
      stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(14), 0);
      currentSpeedSign[1] = 0;
    }
    if (motor & MOTOR3) {
      stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(10), 0);
      stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(11), 0);
      currentSpeedSign[2] = 0;
    }
    if (motor & MOTOR4) {
      stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(5), 0);
      stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(9), 0);
      currentSpeedSign[3] = 0;
    }
  }
}

void motorSetMaxPWM(uint8_t motor, int32_t maxPWM) {
  if (maxPWM < 0)
    return;

  if (motor & MOTOR1) {
    if (maxPWM <= MAX_PWM)
      maxPWMs[0] = maxPWM;
    else
      maxPWMs[0] = MAX_PWM;
  }
  if (motor & MOTOR2) {
    if (maxPWM <= MAX_PWM)
      maxPWMs[1] = maxPWM;
    else
      maxPWMs[1] = MAX_PWM;
  }
  if (motor & MOTOR3) {
    if (maxPWM <= MAX_PWM)
      maxPWMs[2] = maxPWM;
    else
      maxPWMs[2] = MAX_PWM;
  }
  if (motor & MOTOR4) {
    if (maxPWM <= MAX_PWM)
      maxPWMs[3] = maxPWM;
    else
      maxPWMs[3] = MAX_PWM;
  }
}
