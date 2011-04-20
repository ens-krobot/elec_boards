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
#include <drv/timer.h>
#include "trajectory_controller.h"
#include "command_generator.h"
#include "odometry.h"
#include "bezier_utils.h"

#ifndef DD_LINEAR_SPEED_TC
  #define DD_LINEAR_SPEED_TC 0
#endif
#ifndef DD_ROTATIONAL_SPEED_TC
  #define DD_ROTATIONAL_SPEED_TC 1
#endif

#define DD_NO_ERROR 0
#define DD_TRAJECTORY_ALREADY_USED 1

#define DD_GHOST_STOPPED 0
#define DD_GHOST_MOVING 1

/* Initializes the differential drive
 *  - wheel_radius : radius of the wheels (in meters)
 *  - shaft_width : propulsion shaft width (in meters)
 *  - max_wheel_speed : maximum wheel speed (in rad/s)
 *  - v_max : maximum linear speed (in m/s)
 *  - at_max : maximum tangential acceleration (in m/s/s)
 *  - ar_max : maximum radial acceleration (in m/s/s)
 *  - k1, k2, k3 : control loop gain for trajectory following
 *  - Ts : sample time for control loop in seconds
 *
 * Note : the differential drive system will use Trajectory controllers
 *        DD_LINEAR_SPEED_TC and DD_ROTATIONAL_SPEED_TC
 */
void dd_start(float wheel_radius, float shaft_width, float max_wheel_speed,
              float v_max, float at_max, float ar_max,
              float k1, float k2, float k3, float Ts);

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

/*
 * Add a Bezier Spline to the trajectory follower
 */
uint8_t dd_add_bezier(float x_end, float y_end, float d1, float d2, float end_angle, float end_speed);

/*
 * Return the current state of the followed ghost robot
 *  - state : pointer to a robot_state_t structure where the ghost state will be written
 *  - u : pointer to a float in which to write the current value of the parameter on the spline
 *
 * return value :
 *  - DD_GHOST_MOVING : if a trajectory is currently followed
 *  - DD_GHOST_STOPPED : if the ghost robot is stopped
 */
uint8_t dd_get_ghost_state(robot_state_t *state, float *u);

#endif /* __DIFFERENTIAL_DRIVE_H */
