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
 *  - enable_transform : if non-zero, the movement will in world coordinates system
 *  - wheel_radius_xx : radius of the wheels (in meters)
 *  - drive_radius_xx : radius of the drive system (in meters)
 *  - max_wheel_speed : maximum wheel speed (in rad/s)
 *  - Ts : sample time for control loop in seconds
 *
 * Note : the holonomic drive system will use Trajectory controllers
 *        HD_LINEAR_SPEED_{X,Y}_TC and HD_ROTATIONAL_SPEED_TC
 */
void hd_start(uint8_t enable_transform,
              float wheel_radius_f, float wheel_radius_bl, float wheel_radius_br,
              float drive_radius_f, float drive_radius_bl, float drive_radius_br,
              float max_wheel_speed, float target_follow_speed,
              float target_K,
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

/* Move of a given translation
 *  - dx,dy: displacements (in meters)
 */
void hd_move2D(float dx, float dy);

/* Move to a specific location with a target orientation
 *  - x,y: coordinates of the destination (in meters)
 *  - theta: target orientation
 */
void hd_move_to(float x, float y, float theta);

/*
 * Modify a given speed of the robot using the specified acceleration
 */
void hd_set_linear_speed_X(float speed, float acceleration);
void hd_set_linear_speed_Y(float speed, float acceleration);
void hd_set_rotational_speed(float speed, float acceleration);

/* Change limitations of the trajectory follower
 *  - v_lin_max : maximum linear speed (in m/s)
 *  - v_rot_max : maximim rotational speed (in rad/s)
 *  - acc_lin_max : maximum linear acceleration (in m/s/s)
 *  - acc_rot_max : maximum rotational acceleration (in m/s/s)
 */
void hd_adjust_limits(float v_lin_max, float v_rot_max, float acc_lin_max, float acc_rot_max);

/* Lock a target with the holonomic drive at position (target_x, target_y)
 * facing the robot with it orientation 'target_theta'
 */
void hd_lock_target(float target_x, float target_y, float target_theta);

/* Stop locking a target
 */
void hd_unlock_target(void);

float hd_get_lock_error(void);

uint8_t hd_is_lock_enabled_status(void);

#endif /* __HOLONOMIC_DRIVE_H */
