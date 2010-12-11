/*
 * Trajectory planning for the Krobot Junior
 * Xavier Lagorce
 */

#include "planner.h"
#include "can_monitor.h"

Thread *planner;

static msg_t plannerStopThread(void *arg) {

  (void)arg;
  chThdWait(MS2ST(89000));
  stopMatch();

  return 0;
}

static msg_t plannerThread(void *arg) {

  trajectory_t *trajectory;
  arc_t *arc;
  uint8_t i, flags;
  systime_t time, time2;
  int32_t duration;

  (void)arg;

  while (1) {
    trajectory = (trajectory_t*)chMsgWait();

    for (i=0; i < trajectory->arc_nbr; i ++) {
      arc = &(trajectory->path[i]);
      duration = MS2ST(arc->duration);
      if (chThdShouldTerminate())
        chThdExit(0);
      canSetScrew(arc->ptX, arc->ptY, arc->vX, arc->vY, arc->omega);
      time = chTimeNow();
      while (duration > 0) {
        if ((flags=chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(arc->duration))) != 0) {
          time2 = chTimeNow();
          duration -= ((int32_t)time2 - (int32_t)time);
          if (flags & PLANNER_EOG)
            chThdExit(0);
          if (flags & PLANNER_COLLISION) {
            chEvtClear(PLANNER_COLLISION);
            if (trajectory->opt_collisions & COLLISION_STOP) {
              canSetScrew(0, 0, 0, 0, 0);
              chEvtWaitAny(COLLISION_CLEAR);
              chEvtClear(COLLISION_CLEAR);
            }
          }
          time = time2;
        }
        break;
      }
    }
    chMsgRelease(RDY_OK);
  }

  return 0;
}

void TrajectoryPlannerStart(uint8_t opt_time) {

  struct EventListener el0, el1;

  if (planner == NULL) {
    if (opt_time & TIMELIMIT_90S)
      chThdCreateFromHeap(NULL, THD_WA_SIZE(128), HIGHPRIO + 1, plannerStopThread, opt_time);

    // Register collision events
    chEvtRegister(&adcAlarmWarn[ADC_3], &el0, PLANNER_COLLISION);
    chEvtRegister(&adcAlarmOK[ADC_3], &el1, COLLISION_CLEAR);
    adcSetAlarm(ADC_3, 1800, 2200);

    planner = chThdCreateFromHeap(NULL, THD_WA_SIZE(1024), NORMALPRIO + 1, plannerThread, NULL);
  }
}

void followTrajectory(trajectory_t* trajectory, void(*callback)(void)) {
  
  chMsgSend(planner, (msg_t)trajectory);

  if (callback != NULL)
    callback();
}

void stopMatch(void) {
  canSetScrew(0,0,0,0,0);
  chEvtSignal(planner, PLANNER_EOG);
  chThdTerminate(planner);
  chThdWait(planner);
  canSetScrew(0,0,0,0,0);
  planner = NULL;
}
