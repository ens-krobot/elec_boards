/*
 * Speed control
 * Xavier Lagorce
 */

#include "speed_control.h"

#define MAX(x,y) ((x) > (y) ? (x) : (y))
#define MIN(x,y) ((x) > (y) ? (y) : (x))

volatile int32_t ref_speeds[3] = {0, 0, 0};
volatile int32_t cur_speeds[3] = {0, 0, 0};

/*
 * Speed controller thread
 */
static WORKING_AREA(waThreadSC, 512);
static msg_t ThreadSpeedController(void *arg) {

  systime_t time;
  int32_t commands[3] = {0, 0, 0};
  int32_t errors[3] = {0, 0, 0};
  uint16_t positions[3] = {0, 0, 0};
  int32_t last_errors[3] = {0, 0, 0};
  uint16_t prev_positions[3] = {0, 0, 0};
  int32_t integ[3] = {0, 0, 0};


  (void)arg;
  time = chTimeNow();

  while (TRUE) {
    time += MS2ST(Tcomp);

    prev_positions[0] = getEncoderPosition(ENCODER1);
    prev_positions[1] = getEncoderPosition(ENCODER2);
    prev_positions[2] = getEncoderPosition(ENCODER3);

    time += MS2ST(Te-Tcomp);
    chThdSleepUntil(time);

    positions[0] = getEncoderPosition(ENCODER1);
    positions[1] = getEncoderPosition(ENCODER2);
    positions[2] = getEncoderPosition(ENCODER3);

    cur_speeds[0] = (K_v*((int32_t)positions[0] - (int32_t)prev_positions[0]));
    cur_speeds[1] = (K_v*((int32_t)positions[1] - (int32_t)prev_positions[1]));
    cur_speeds[2] = (K_v*((int32_t)positions[2] - (int32_t)prev_positions[2]));

    errors[0] = ref_speeds[0] - cur_speeds[0];
    errors[1] = ref_speeds[1] - cur_speeds[1];
    errors[2] = ref_speeds[2] - cur_speeds[2];

    integ[0] += (errors[0] + last_errors[0])*Te/2;
    integ[1] += (errors[1] + last_errors[1])*Te/2;
    integ[2] += (errors[2] + last_errors[2])*Te/2;

    if (integ[0] > INTEG_MAX)
      integ[0] = INTEG_MAX;
    else if (integ[0] < -INTEG_MAX)
      integ[0] = -INTEG_MAX;
    if (integ[1] > INTEG_MAX)
      integ[1] = INTEG_MAX;
    else if (integ[1] < -INTEG_MAX)
      integ[1] = -INTEG_MAX;
    if (integ[2] > INTEG_MAX)
      integ[2] = INTEG_MAX;
    else if (integ[2] < -INTEG_MAX)
      integ[2] = -INTEG_MAX;

    //--> Command computation
    commands[0] = (K_P*errors[0] + K_I*integ[0])/100;
    commands[1] = (K_P*errors[1] + K_I*integ[1])/100;
    commands[2] = (K_P*errors[2] + K_I*integ[2])/100;
    //--> End of command computation
    
    last_errors[0] = errors[0];
    last_errors[1] = errors[1];
    last_errors[2] = errors[2];
    
    motorSetSpeed(MOTOR1, commands[0]);
    motorSetSpeed(MOTOR2, commands[1]);
    motorSetSpeed(MOTOR3, commands[2]);
    
    prev_positions[0] = positions[0];
    prev_positions[1] = positions[1];
    prev_positions[2] = positions[2];

    chThdSleepUntil(time);
  }
  return 0;
}


void speedControlInit(void) {

  encodersInit();
  motorsInit();

  enableMotor(MOTOR1 | MOTOR2 | MOTOR3);
  motorSetSpeed(MOTOR1 | MOTOR2 | MOTOR3, 0);
  resetEncoderPosition(ENCODER1 | ENCODER2 | ENCODER3);

  chThdCreateStatic(waThreadSC, sizeof(waThreadSC), HIGHPRIO, ThreadSpeedController, NULL);
}

void sc_setRefSpeed(uint8_t motor, int32_t speed) {

  if (motor & MOTOR1) {
    ref_speeds[0] = speed;
  }
  if (motor & MOTOR2) {
    ref_speeds[1] = speed;
  }
  if (motor & MOTOR3) {
    ref_speeds[2] = speed;
  }
}

int32_t sc_getRealSpeed(uint8_t motor) {
  
  switch(motor) {
    case MOTOR1:
      return cur_speeds[0];
      break;
    case MOTOR2:
      return cur_speeds[1];
      break;
    case MOTOR3:
      return cur_speeds[2];
      break;
    default:
      return 0;
  }
}
