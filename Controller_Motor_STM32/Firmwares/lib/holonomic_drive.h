/*
 * holonomic_drive.h
 * --------------------
 * Copyright : (c) 2011, Xavier Lagorce <Xavier.Lagorce@crans.org>
 * Licence   : BSD3
 *
 * This file is a part of [kro]bot.
 */

#ifndef __HOLONOMIC_DRIVE_H
#define __HOLONOMIC_DRIVE_H

#include <math.h>
#include <drv/timer.h>
#include "trajectory_controller.h"
#include "command_generator.h"
#include "odometry_holonomic.h"

#define HD_LINEAR_SPEED_X_TC 0
#define HD_LINEAR_SPEED_Y_TC 1
#define HD_ROTATIONAL_SPEED_TC 2

/* Initializes the holonomic drive
 *  - odometry_process : ID of the odometry process to base the control on
 *  - wheel_radius : radius of the wheels (in meters)
 *  - drive_radius : radius of the drive system (in meters)
 *  - max_wheel_speed : maximum wheel speed (in rad/s)
 *  - Ts : sample time for control loop in seconds
 *
 * Note : the holonomic drive system will use Trajectory controllers
 *        HD_LINEAR_SPEED_{X,Y}_TC and HD_ROTATIONAL_SPEED_TC
 */
void hd_start(uint8_t odometry_process,
              float wheel_radius, float drive_radius,
              float max_wheel_speed,
              float Ts);

/* Pauses or Resumes the holonomic drive system.
 * In pause mode, the drive will accept no further command and actions will be
 * stopped.
 * If the robot is moving, wheels will be slowed down with the last used
 * acceleration (note that this won't necessarily give a straight line) when
 * pausing.
 */
void hd_pause(void);
void hd_resume(void);

/*
 * Stops the differential drive system.
 */
void hd_stop(void);

/* Gets the generator correspondig to the wheels positions
 * to use them in motor_control for instance
 */
command_generator_t* hd_get_front_wheel_generator(void);
command_generator_t* hd_get_back_right_wheel_generator(void);
command_generator_t* hd_get_back_left_wheel_generator(void);

/* Moves along a line
 *  - distance : distance to move of (in meters), can be positive (forward)
 *               or negative (backward).
 *  - speed : moving speed (in meters per second) should be positive.
 *  - acceleration : in meters per second per second, should be positive.
 */
void hd_move_X(float distance, float speed, float acceleration);
void hd_move_Y(float distance, float speed, float acceleration);

/* Turns around the drive center
 *  - angle : angle to turn of (in degrees), can be positive (CCW)
 *                or negative (CW).
 *  - speed : turning speed (in degrees per second) should be positive.
 *  - acceleration : in degrees per second per second, should be positive.
 */
void hd_turn(float angle, float speed, float acceleration);

/*
 * Modify a given speed of the robot using the specified acceleration
 */
void hd_set_linear_speed_X(float speed, float acceleration);
void hd_set_linear_speed_Y(float speed, float acceleration);
void hd_set_rotational_speed(float speed, float acceleration);

#endif /* __HOLONOMIC_DRIVE_H */
