/*
 * lift_controller.h
 * -----------------
 * Copyright : (c) 2011, Xavier Lagorce <Xavier.Lagorce@crans.org>
 * Licence   : BSD3
 *
 * This file is a part of [kro]bot.
 */

#ifndef __LIFT_CONTROLLER_H
#define __LIFT_CONTROLLER_H

#include "motor_controller.h"
#include "trajectory_controller.h"
#include "command_generator.h"

#define LC_FRONT_LIFT  0
#define LC_BACK_LIFT   1

#define LC_FRONT_UP     1
#define LC_FRONT_BOTTOM 2
#define LC_BACK_UP      4
#define LC_BACK_BOTTOM  8

#define LC_UP     1
#define LC_BOTTOM 2

#define LC_POSITION_BOTTOM 1
#define LC_POSITION_MIDDLE 2
#define LC_POSITION_UP     4

#define LC_TC_FRONT 0
#define LC_TC_BACK 1

void lc_init(void);
void lc_end_stop_reached(uint8_t end_stops);
void lc_homing(uint8_t lift);
void lc_goto_position(uint8_t lift, float position);
void lc_release(void);

#endif /* __LIFT_CONTROLLER_H */
