/*
 * odometry.h
 * ----------
 * Copyright : (c) 2011, Xavier Lagorce <Xavier.Lagorce@crans.org>
 * Licence   : BSD3
 *
 * This file is a part of [kro]bot.
 */

#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include <math.h>
#include <drv/timer.h>
#include "encoder.h"

typedef struct {
  float x;
  float y;
  float theta;
} robot_state_t;

void odometryInit(float Ts, float wheel_radius, float shaft_width, float encoder_gain);
void odo_disable(void);
void odo_restart(void);

void odo_setState(robot_state_t *new_state);

void odo_getState(robot_state_t *robot_state);

#endif /* __ODOMETRY_H */
