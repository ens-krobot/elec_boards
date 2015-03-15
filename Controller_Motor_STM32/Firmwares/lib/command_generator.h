/*
 * command_generator.h
 * -------------------
 * Copyright : (c) 2011, Xavier Lagorce <Xavier.Lagorce@crans.org>
 * Licence   : BSD3
 *
 * This module aims at providing a simple way to generate some command signal
 * for control processes (like steps, ramps, ...)
 *
 * This file is a part of [kro]bot.
 */

#ifndef __COMMAND_GENERATOR_H
#define __COMMAND_GENERATOR_H

#include <drv/timer.h>
#include <math.h>
#include "encoder.h"
#include "odometry_holonomic.h"

// Generator types
#define GEN_NONE     0   // No type, the generator is not initialized.
#define GEN_CONSTANT 1   // Outputs a constant.
#define GEN_RAMP     2   // Outputs a ramp with the given slope.
#define GEN_RAMP2    3   // Same as GEN_RAMP, but the slope is given by another
                         // generator.
#define GEN_DD_LEFT  4   // Outputs the left wheel speed for a differential
                         // drive.
#define GEN_DD_RIGHT 5   // Outputs the right wheel speed for a differential
                         // drive.
#define GEN_HD_F     6   // Outputs the front wheel speed for an holonomic drive.
#define GEN_HD_BR    7   // Outputs the back-right wheel speed for an holonomic drive.
#define GEN_HD_BL    8   // Outputs the back-left wheel speed for an holonomic drive.
#define GEN_AC_T1    9   // Outputs the angle of the first articulation of a RR arm
#define GEN_AC_T2   10   // Outputs the angle of the second articulation of a RR arm


// Generator states
#define GEN_STATE_PAUSE   0  // The output is freezed.
#define GEN_STATE_RUNNING 1  // The generator is running.

// Threshold comparison for callback trigger
#define GEN_CALLBACK_NONE 0  // There is no callback.
#define GEN_CALLBACK_SUP 1   // The callback is triggered if output > threshold.
#define GEN_CALLBACK_INF 2   // The callback is triggered if output < threshold.

typedef union _command_generator_t command_generator_t;

// Callback description
typedef struct {
  uint8_t type;
  float threshold;
  void (*callback_function)(command_generator_t*);
} generator_callback_t;

// Generator descriptions
typedef struct {
  uint8_t t;
  float last_output;
  uint8_t state;
  generator_callback_t callback;
} placeholder_generator_t;

typedef struct {
  placeholder_generator_t gen;
} constant_generator_t;

typedef struct {
  placeholder_generator_t gen;
  int32_t last_time;
  float speed;
} ramp_generator_t;

typedef struct {
  placeholder_generator_t gen;
  int32_t last_time;
  command_generator_t *speed;
} ramp2_generator_t;

typedef struct {
  placeholder_generator_t gen;
  command_generator_t *linear_pos;
  command_generator_t *linear_speed;
  command_generator_t *rotational_pos;
  command_generator_t *rotational_speed;
  float wheel_radius;
  float shaft_width;
  float max_speed;
} dd_generator_t;

typedef struct {
  placeholder_generator_t gen;
  command_generator_t *linear_pos_x;
  command_generator_t *linear_speed_x;
  command_generator_t *linear_pos_y;
  command_generator_t *linear_speed_y;
  command_generator_t *rotational_pos;
  command_generator_t *rotational_speed;
  float wheel_radius;
  float struct_radius;
  float max_speed;
  uint8_t enable_transform;
} hd_generator_t;

typedef struct {
  placeholder_generator_t gen;
  command_generator_t *linear_pos_x;
  command_generator_t *linear_speed_x;
  command_generator_t *linear_pos_y;
  command_generator_t *linear_speed_y;
  float l1;
  float l2;
  uint8_t enc_theta1;
  uint8_t enc_theta2;
} a2r_generator_t;

// Usable generator meta-type
union _command_generator_t {
  placeholder_generator_t type;
  constant_generator_t constant;
  ramp_generator_t ramp;
  ramp2_generator_t ramp2;
  dd_generator_t dd;
  hd_generator_t hd;
  a2r_generator_t a2r;
};

/* Initializes a new Constant Generator.
 *  - generator : pointer to the generator to initialize
 *  - value : output value of the generator
 */
command_generator_t* new_constant_generator(command_generator_t *generator,
                                            float value);

/* Initializes a new Ramp Generator.
 *  - generator : pointer to the generator to initialize
 *  - starting_value : initial output value
 *  - speed : slope
 */
command_generator_t* new_ramp_generator(command_generator_t *generator,
                                        float starting_value, float speed);

/* Initializes a new Ramp2 Generator.
 *  - generator : pointer to the generator to initialize
 *  - starting_value : initial output value
 *  - speed : pointer to the generator which output is used as this generator's slope
 */
command_generator_t* new_ramp2_generator(command_generator_t *generator,
                                         float starting_value,
                                         command_generator_t *speed);

/* Initializes a new Differential Drive generator.
 *  - generator : pointer to the generator to initialize
 *  - linear_pos : pointer to the generator giving the integrates of linear_speed. This
 *                 generator will be called at each computation to allow update in parallel
 *                 with linear_speed.
 *  - linear_speed : pointer to the generator giving the linear speed of the drive
 *  - rotational_pos : pointer to the generator giving the integrates of rotational_speed. This
 *                 generator will be called at each computation to allow update in parallel
 *                 with rotational_speed.
 *  - rotational_speed : pointer to the generator giving the rotational speed of the drive
 *  - wheel_radius : radius of the wheels
 *  - shaft_width : width of the propulsion shaft
 *  - max_speed : maximum wheel speed (in rad/s)
 *  - type : 1 for the right_wheel, -1 for the left_wheel
 */
command_generator_t* new_dd_generator(command_generator_t *generator,
                                      command_generator_t *linear_pos,
                                      command_generator_t *linear_speed,
                                      command_generator_t *rotational_pos,
                                      command_generator_t *rotational_speed,
                                      float wheel_radius, float shaft_width, float max_speed,
                                      uint8_t type);

/* Initializes a new Holonomic Drive generator.
 *  - generator : pointer to the generator to initialize
 *  - linear_pos_x : pointer to the generator giving the integrates of linear_speed_x. This
 *                   generator will be called at each computation to allow update in parallel
 *                   with linear_speed.
 *  - linear_speed_x : pointer to the generator giving the linear speed along the x axis of
 *                     the drive.
 *  - linear_pos_y : pointer to the generator giving the integrates of linear_speed_y. This
 *                   generator will be called at each computation to allow update in parallel
 *                   with linear_speed.
 *  - linear_speed_y : pointer to the generator giving the linear speed along the y axis of
 *                     the drive.
 *  - rotational_pos : pointer to the generator giving the integrates of rotational_speed. This
 *                     generator will be called at each computation to allow update in parallel
 *                     with rotational_speed.
 *  - rotational_speed : pointer to the generator giving the rotational speed of the drive.
 *  - wheel_radius : radius of the wheels.
 *  - struct_radius : radius of the holonomic drive.
 *  - max_speed : maximum wheel speed (in rad/s).
 *  - type :
 *     o 1 is the front wheel
 *     o 2 is the back-right wheel
 *     o 3 is the back-left wheel
 */
command_generator_t* new_hd_generator(command_generator_t *generator,
                                      command_generator_t *linear_pos_x,
                                      command_generator_t *linear_speed_x,
                                      command_generator_t *linear_pos_y,
                                      command_generator_t *linear_speed_y,
                                      command_generator_t *rotational_pos,
                                      command_generator_t *rotational_speed,
                                      float wheel_radius, float struct_radius, float max_speed,
                                      uint8_t enable_transform, uint8_t type);

/* Initializes a new 2R arm controller generator.
 *  - generator : pointer to the generator to initialize
 *  - linear_pos_x : pointer to the generator giving the integrates of linear_speed_x. This
 *                   generator will be called at each computation to allow update in parallel
 *                   with linear_speed.
 *  - linear_speed_x : pointer to the generator giving the linear speed along the x axis of
 *                     the arm end.
 *  - linear_pos_y : pointer to the generator giving the integrates of linear_speed_y. This
 *                   generator will be called at each computation to allow update in parallel
 *                   with linear_speed.
 *  - linear_speed_y : pointer to the generator giving the linear speed along the y axis of
 *                     the arm end.
 *  - l1 : length of the first segment of the arm.
 *  - l2 : length of the second segment of the arm.
 *  - enc_theta1 : ID of the encoder measuring the first articulation position
 *  - enc_theta2 : ID of the encoder measuring the second articulation position
 *  - type :
 *     o 1 is the first articulation
 *     o 2 is the second articulation
 */
command_generator_t* new_a2r_generator(command_generator_t *generator,
                                       command_generator_t *linear_pos_x,
                                       command_generator_t *linear_speed_x,
                                       command_generator_t *linear_pos_y,
                                       command_generator_t *linear_speed_y,
                                       float l1, float l2,
                                       uint8_t enc_theta1, uint8_t enc_theta2,
                                       uint8_t type);

/*
 * Adjusts the current output value of 'generator' to 'value'.
 */
command_generator_t* adjust_value(command_generator_t *generator, float value);

/*
 * Adjusts the current slope of the ramp generator 'generator' to 'speed'.
 */
command_generator_t* adjust_speed(command_generator_t *generator, float speed);

/*
 * Starts or pauses 'generator'.
 */
command_generator_t* start_generator(command_generator_t *generator);
command_generator_t* pause_generator(command_generator_t *generator);

/* Adds a callback to a generator
 *  - generator : pointer to the generator to add a callback to
 *  - type : threshold comparison type
 *  - threshold : comparison threshold
 *  - callback_function : callback to be called when the event is triggered
 */
command_generator_t* add_callback(command_generator_t *generator, uint8_t type, float threshold, void (*callback_function)(command_generator_t*));

/*
 * Removes the callback from 'generator'
 */
command_generator_t* remove_callback(command_generator_t *generator);

/*
 * Gets (and computes) the current output value of 'generator'
 */
float get_output_value(command_generator_t *generator);

#endif /* __COMMAND_GENERATOR_H */
