/*
 * Motor controller interface
 * Xavier Lagorce
 */

#include "motor.h"

uint8_t enabledMotors = 0;
signed char currentSpeedSign[] = {0, 0, 0};

/*
 * Interrupt routine for PWM generation
 */
void VectorB0(void) {

  if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
  {
    // Clear TIM2 Capture Compare1 interrupt pending bit
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);

    // End of motor1 pulse
    GPIO_ResetBits(GPIOC, GPIO_Pin_13);
  }
  else if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)
  {
    // Clear TIM2 Capture Compare2 interrupt pending bit
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);

    // End of motor2 pulse
    GPIO_ResetBits(GPIOC, GPIO_Pin_3);
  }
  else if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)
  {
    // Clear TIM2 Capture Compare3 interrupt pending bit
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);

    // End of motor3 pulse
    GPIO_ResetBits(GPIOC, GPIO_Pin_9);
  }
  else {
    // TIM_IT_Update
    
    // Clear TIM2 Update interrupt pending bit
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

    GPIO_SetBits(GPIOC, GPIO_Pin_3 | GPIO_Pin_9 | GPIO_Pin_13);
  }
}

/*
 * Initialises TIM2 for PWM generation and associated GPIOs
 */
void motorsInit(void) {

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef       TIM_OCInitStructure;
  GPIO_InitTypeDef        GPIO_InitStructure;
  
  //Enable GPIOB and GPIOC clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  
  //Enable timer clock
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  
  //Setup timer for quadrature encoder interface
  //Motor1 : STBY PC10
  //         IN1  PB0
  //         IN2  PB1
  //         PWM  PC13
  //Motor2 : STBY PC0
  //         IN1  PC1
  //         IN2  PC2
  //         PWM  PC3
  //Motor3 : STBY PC6
  //         IN1  PC7
  //         IN2  PC8
  //         PWM  PC9
  GPIO_InitStructure.GPIO_Pin     = (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6
                                     | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_13);
  GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_0|GPIO_Pin_1;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Default value of H-Bridge configuration
  GPIO_ResetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1);
  GPIO_ResetBits(GPIOC, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_10
                           | GPIO_Pin_3 | GPIO_Pin_9 | GPIO_Pin_13);

  // TimeBase configuration
  TIM_TimeBaseStructure.TIM_Prescaler     = 0;
  TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period        = 36000; // 20 kHz
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  
  // Output Compare Active Mode configuration: Channel1
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Inactive;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_Pulse = 18000;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM2, &TIM_OCInitStructure);
  TIM_OC2Init(TIM2, &TIM_OCInitStructure);
  TIM_OC3Init(TIM2, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);
  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Disable);

  // Enable interrupt vector
  NVICEnableVector(TIM2_IRQn, 0);

  // TIM IT enable
  TIM_ITConfig(TIM2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_Update, ENABLE);

  // All motors are disabled
  enabledMotors = 0;

  //Enable timer Peripherals
  TIM_Cmd(TIM2,ENABLE);
}

/*
 * Enable a motor driver
 */
void enableMotor(uint8_t motor) {

  if (motor & MOTOR1) {
    enabledMotors |= MOTOR1;
    GPIO_SetBits(GPIOC, GPIO_Pin_10);
  }
  if (motor & MOTOR2) {
    enabledMotors |= MOTOR2;
    GPIO_SetBits(GPIOC, GPIO_Pin_0);
  }
  if (motor & MOTOR3) {
    enabledMotors |= MOTOR3;
    GPIO_SetBits(GPIOC, GPIO_Pin_6);
  }
}

/*
 * Disable a motor driver
 */
void disableMotor(uint8_t motor) {

  if (motor & MOTOR1) {
    enabledMotors &= ~MOTOR1;
    GPIO_ResetBits(GPIOC, GPIO_Pin_10);
  }
  if (motor & MOTOR2) {
    enabledMotors &= ~MOTOR2;
    GPIO_ResetBits(GPIOC, GPIO_Pin_0);
  }
  if (motor & MOTOR3) {
    enabledMotors &= ~MOTOR3;
    GPIO_ResetBits(GPIOC, GPIO_Pin_6);
  }
}

/*
 * Step one motor's speed
 */
void motorSetSpeed(uint8_t motor, int32_t speed) {

  if (speed == 0) {
    motorStop(motor, MOTOR_BRAKE);
    return;
  }

  if (speed >= MAX_PWM)
    speed = MAX_PWM;
  if (speed <= -MAX_PWM)
    speed = -MAX_PWM;

  if (motor & MOTOR1) {
    if(speed > 0) {
      if (currentSpeedSign[0] != 1) {
        currentSpeedSign[0] = 1;
        GPIO_SetBits(GPIOB, GPIO_Pin_1);
        GPIO_ResetBits(GPIOB, GPIO_Pin_0);
      }
      TIM_SetCompare1(TIM2, (uint16_t)speed);
    } else {
      if (currentSpeedSign[0] != -1) {
        currentSpeedSign[0] = -1;
        GPIO_SetBits(GPIOB, GPIO_Pin_0);
        GPIO_ResetBits(GPIOB, GPIO_Pin_1);
      }
      TIM_SetCompare1(TIM2, (uint16_t)(-speed));
    }
  }
  if (motor & MOTOR2) {
    if(speed > 0) {
      if (currentSpeedSign[1] != 1) {
        currentSpeedSign[1] = 1;
        GPIO_SetBits(GPIOC, GPIO_Pin_2);
        GPIO_ResetBits(GPIOC, GPIO_Pin_1);
      }
      TIM_SetCompare2(TIM2, (uint16_t)speed);
    } else {
      if (currentSpeedSign[1] != -1) {
        currentSpeedSign[1] = -1;
        GPIO_SetBits(GPIOC, GPIO_Pin_1);
        GPIO_ResetBits(GPIOC, GPIO_Pin_2);
      }
      TIM_SetCompare2(TIM2, (uint16_t)(-speed));
    }
  }
  if (motor & MOTOR3) {
    if(speed > 0) {
      if (currentSpeedSign[2] != 1) {
        currentSpeedSign[2] = 1;
        GPIO_SetBits(GPIOC, GPIO_Pin_8);
        GPIO_ResetBits(GPIOC, GPIO_Pin_7);
      }
      TIM_SetCompare3(TIM2, (uint16_t)speed);
    } else {
      if (currentSpeedSign[2] != -1) {
        currentSpeedSign[2] = -1;
        GPIO_SetBits(GPIOC, GPIO_Pin_7);
        GPIO_ResetBits(GPIOC, GPIO_Pin_8);
      }
      TIM_SetCompare3(TIM2, (uint16_t)(-speed));
    }
  }
}

/*
 * Stop a motor using the 'mode' mode.
 */
void motorStop(uint8_t motor, uint8_t mode) {

  if (mode == MOTOR_STOP) {
    if (motor & MOTOR1) {
      GPIO_ResetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1);
      currentSpeedSign[0] = 0;
    }
    if (motor & MOTOR2) {
      GPIO_ResetBits(GPIOC, GPIO_Pin_1 | GPIO_Pin_2);
      currentSpeedSign[1] = 0;
    }
    if (motor & MOTOR3) {
      GPIO_ResetBits(GPIOC, GPIO_Pin_7 | GPIO_Pin_8);
      currentSpeedSign[2] = 0;
    }
  } else if (mode == MOTOR_BRAKE) {
    if (motor & MOTOR1) {
      GPIO_SetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1);
      currentSpeedSign[0] = 0;
    }
    if (motor & MOTOR2) {
      GPIO_SetBits(GPIOC, GPIO_Pin_1 | GPIO_Pin_2);
      currentSpeedSign[1] = 0;
    }
    if (motor & MOTOR3) {
      GPIO_SetBits(GPIOC, GPIO_Pin_7 | GPIO_Pin_8);
      currentSpeedSign[2] = 0;
    }    
  }
}
