/*
 * odometry_holonomic.h
 * --------------------
 * Copyright : (c) 2015, Xavier Lagorce <Xavier.Lagorce@crans.org>
 * Licence   : BSD3
 *
 * This file is a part of [kro]bot.
 */

#ifndef __ODOMETRY_HOLONOMIC_H
#define __ODOMETRY_HOLONOMIC_H

#include <math.h>
#include <drv/timer.h>
#include "encoder.h"
#include "odometry.h"

// Odometry initialization for differential drive
void HolonomicOdometryInit(float Ts,
                           float wheel_radius_f, float wheel_radius_bl, float wheel_radius_br,
                           float drive_radius_f, float drive_radius_bl, float drive_radius_br,
                           uint8_t front_encoder, uint8_t back_left_encoder, uint8_t back_right_encoder,
                           float encoder_gain_f, float encoder_gain_bl, float encoder_gain_br);
void HolOdo_disable(void);
void HolOdo_restart(void);

void HolOdo_setState(robot_state_t *new_state);

void HolOdo_getState(robot_state_t *robot_state);

float HolOdo_getTheta(void);

#endif /* __ODOMETRY_HOLONOMIC_H */
