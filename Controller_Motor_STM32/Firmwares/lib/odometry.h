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

#define MAX_ODOMETRY_PROCESSES 2

typedef struct {
  float x;
  float y;
  float theta;
} robot_state_t;

void odometryInit(uint8_t process_num, float Ts,
                  float left_wheel_radius, float right_wheel_radius,
                  float shaft_width,
                  uint8_t left_encoder, uint8_t right_encoder,
                  float left_encoder_gain, float right_encoder_gain);
void odo_disable(uint8_t process_num);
void odo_restart(uint8_t process_num);

void odo_setState(uint8_t process_num, robot_state_t *new_state);

void odo_getState(uint8_t process_num, robot_state_t *robot_state);

#endif /* __ODOMETRY_H */
