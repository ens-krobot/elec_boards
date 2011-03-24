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

#define GEN_NONE     0
#define GEN_CONSTANT 1
#define GEN_RAMP     2
#define GEN_RAMP2    3

#define GEN_STATE_PAUSE   0
#define GEN_STATE_RUNNING 1

typedef union _command_generator_t command_generator_t;

typedef struct {
  uint8_t t;
  float last_output;
  uint8_t state;
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

union _command_generator_t {
  placeholder_generator_t type;
  constant_generator_t constant;
  ramp_generator_t ramp;
  ramp2_generator_t ramp2;
};

command_generator_t* new_constant_generator(command_generator_t *generator, float value);
command_generator_t* new_ramp_generator(command_generator_t *generator, float starting_value, float speed);
command_generator_t* new_ramp2_generator(command_generator_t *generator, float starting_value, command_generator_t *speed);

command_generator_t* adjust_value(command_generator_t *generator, float value);
command_generator_t* adjust_speed(command_generator_t *generator, float speed);

command_generator_t* start_generator(command_generator_t *generator);
command_generator_t* pause_generator(command_generator_t *generator);

float get_output_value(command_generator_t *generator);

#endif /* __COMMAND_GENERATOR_H */
