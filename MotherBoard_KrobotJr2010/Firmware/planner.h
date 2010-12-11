/*
 * Trajectory planning for the Krobot Junior
 * Xavier Lagorce
 */

#ifndef HEADER__PLANNER
#define HEADER__PLANNER

#define TIMELIMIT_NONE    0
#define TIMELIMIT_90S     1

#define COLLISION_STOP    1
#define COLLISION_IGNORE  0

#define PLANNER_ABORT     1
#define PLANNER_EOG       2
#define PLANNER_COLLISION 4
#define COLLISION_CLEAR   8

#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "can_monitor.h"
#include "watch_adc.h"

extern Thread *planner;

typedef struct {
  int16_t ptX;
  int16_t ptY;
  int16_t vX;
  int16_t vY;
  int16_t omega;
  uint16_t duration;
} arc_t;

typedef struct {
  uint8_t arc_nbr;
  uint8_t opt_collisions;
  arc_t* path;
} trajectory_t;

void TrajectoryPlannerStart(uint8_t opt_time);
void followTrajectory(trajectory_t* trajectory, void(*callback)(void));
void stopMatch(void);

#endif
