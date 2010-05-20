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
  int32_t last_commands[3] = {0, 0, 0};
  uint16_t prev_positions[3] = {0, 0, 0};


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

    //--> Command computation
    commands[0] = ((K_Pn + K_In)*errors[0] - K_Pn*last_errors[0] + 100*last_commands[0])/100;
    commands[1] = ((K_Pn + K_In)*errors[1] - K_Pn*last_errors[1] + 100*last_commands[1])/100;
    commands[2] = ((K_Pn + K_In)*errors[2] - K_Pn*last_errors[2] + 100*last_commands[2])/100;
    //--> End of command computation
    
    if (commands[0] >= 0)
      commands[0] = MIN(MAX_COMMAND, commands[0]);
    else
      commands[0] = MAX(-MAX_COMMAND, commands[0]);
    if (commands[1] >= 0)
      commands[1] = MIN(MAX_COMMAND, commands[1]);
    else
      commands[1] = MAX(-MAX_COMMAND, commands[1]);
    if (commands[2] >= 0)
      commands[2] = MIN(MAX_COMMAND, commands[2]);
    else
      commands[2] = MAX(-MAX_COMMAND, commands[2]);
      
    last_commands[0] = commands[0];
    last_commands[1] = commands[1];
    last_commands[2] = commands[2];
      
    if (commands[0] >= -DEAD_ZONE && commands[0] <= DEAD_ZONE)
      commands[0] = 0;
    if (commands[1] >= -DEAD_ZONE && commands[1] <= DEAD_ZONE)
      commands[1] = 0;
    if (commands[2] >= -DEAD_ZONE && commands[2] <= DEAD_ZONE)
      commands[2] = 0;
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
