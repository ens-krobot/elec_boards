/*
 * differential_drive.h
 * --------------------
 * Copyright : (c) 2011, Xavier Lagorce <Xavier.Lagorce@crans.org>
 * Licence   : BSD3
 *
 * This file is a part of [kro]bot.
 */

#ifndef __DIFFERENTIAL_DRIVE_H
#define __DIFFERENTIAL_DRIVE_H

#include <math.h>
#include "trajectory_controller.h"
#include "command_generator.h"

#ifndef DD_LINEAR_SPEED_TC
  #define DD_LINEAR_SPEED_TC 0
#endif
#ifndef DD_ROTATIONAL_SPEED_TC
  #define DD_ROTATIONAL_SPEED_TC 1
#endif

/* Initializes the differential drive
 *  - wheel_radius : radius of the wheels (in meters)
 *  - shaft_width : propulsion shaft radius (in meters)
 *  - max_speed : maximum wheel speed (in rad/s)
 *
 * Note : the differential drive system will use Trajectory controllers
 *        DD_LINEAR_SPEED_TC and DD_ROTATIONAL_SPEED_TC
 */
void dd_start(float wheel_radius, float shaft_width, float max_speed);

/* Pauses or Resumes the differential drive system.
 * In pause mode, the drive will accept no further command and actions will be
 * stopped.
 * If the robot is moving, wheels will be slowed down with the last used
 * acceleration (note that this won't necessarily give a straight line) when
 * pausing.
 */
void dd_pause(void);
void dd_resume(void);

/*
 * Stops the differential drive system.
 */
void dd_stop(void);

/* Gets the generator correspondig to the wheels positions
 * to use them in motor_control for instance
 */
command_generator_t* dd_get_left_wheel_generator(void);
command_generator_t* dd_get_right_wheel_generator(void);

/* Moves along a line
 *  - distance : distance to move of (in meters), can be positive (forward)
 *                or negative (backward).
 *  - speed : moving speed (in meters per second) should be positive.
 *  - acceleration : in meters per second per second, should be positive.
 */
void dd_move(float distance, float speed, float acceleration);

/* Turns around the propulsion shaft center
 *  - angle : angle to turn of (in degrees), can be positive (CCW)
 *                or negative (CW).
 *  - speed : turning speed (in degrees per second) should be positive.
 *  - acceleration : in degrees per second per second, should be positive.
 */
void dd_turn(float angle, float speed, float acceleration);

/*
 * Modify a given speed of the robot using the specified acceleration
 */
void dd_set_linear_speed(float speed, float acceleration);
void dd_set_rotational_speed(float speed, float acceleration);

#endif /* __DIFFERENTIAL_DRIVE_H */
