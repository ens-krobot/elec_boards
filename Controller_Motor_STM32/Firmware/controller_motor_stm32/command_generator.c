/*
 * command_generator.c
 * -------------------
 * Copyright : (c) 2011, Xavier Lagorce <Xavier.Lagorce@crans.org>
 * Licence   : BSD3
 *
 * This file is a part of [kro]bot.
 */

#include "command_generator.h"


command_generator_t* new_constant_generator(command_generator_t *generator, float value) {
  generator->type.t = GEN_CONSTANT;
  generator->type.callback.type = GEN_CALLBACK_NONE;
  generator->constant.gen.last_output = value;
  generator->constant.gen.state = GEN_STATE_PAUSE;

  return generator;
}

command_generator_t* new_ramp_generator(command_generator_t *generator, float starting_value, float speed) {
  generator->type.t = GEN_RAMP;
  generator->type.callback.type = GEN_CALLBACK_NONE;
  generator->ramp.gen.state = GEN_STATE_PAUSE;
  generator->ramp.gen.last_output = starting_value;
  generator->ramp.last_time = 0;
  generator->ramp.speed = speed;

  return generator;
}

command_generator_t* new_ramp2_generator(command_generator_t *generator, float starting_value, command_generator_t *speed) {
  generator->type.t = GEN_RAMP2;
  generator->type.callback.type = GEN_CALLBACK_NONE;
  generator->ramp2.gen.state = GEN_STATE_PAUSE;
  generator->ramp2.gen.last_output = starting_value;
  generator->ramp2.last_time = 0;
  generator->ramp2.speed = speed;

  return generator;
}


command_generator_t* adjust_value(command_generator_t *generator, float value) {
  generator->type.last_output = value;

  return generator;
}

command_generator_t* adjust_speed(command_generator_t *generator, float speed) {
  switch (generator->type.t) {
  case GEN_RAMP:
    generator->ramp.speed = speed;
    return generator;
  default:
    return NULL;
  }
}

command_generator_t* start_generator(command_generator_t *generator) {
  switch (generator->type.t) {
  case GEN_RAMP:
    generator->ramp.last_time = ticks_to_us(timer_clock());
    break;
  case GEN_RAMP2:
    start_generator(generator->ramp2.speed);
    generator->ramp2.last_time = ticks_to_us(timer_clock());
  }
  generator->type.state = GEN_STATE_RUNNING;

  return generator;
}

command_generator_t* pause_generator(command_generator_t *generator) {
  // update generator->type.last_output
  get_output_value(generator);
  // pause the generator
  generator->type.state = GEN_STATE_PAUSE;

  if (generator->type.t == GEN_RAMP2)
    pause_generator(generator->ramp2.speed);

  return generator;
}

command_generator_t* add_callback(command_generator_t *generator, uint8_t type, float threshold, void (*callback_function)(command_generator_t*)) {
  generator->type.callback.callback_function = callback_function;
  generator->type.callback.threshold = threshold;
  generator->type.callback.type = type;

  return generator;
}

command_generator_t* remove_callback(command_generator_t *generator) {
  generator->type.callback.type = GEN_CALLBACK_NONE;

  return generator;
}


float get_output_value(command_generator_t *generator) {
  int32_t cur_time;
  float speed, dt;

  switch (generator->type.t) {
  case GEN_CONSTANT:
    // Constant generator, no update needed
    break;
  case GEN_RAMP:
    cur_time = ticks_to_us(timer_clock());
    dt = (cur_time - generator->ramp.last_time)*1e-6;
    generator->type.last_output += dt*generator->ramp.speed;
    generator->ramp.last_time = cur_time;
  case GEN_RAMP2:
    cur_time = ticks_to_us(timer_clock());
    speed = get_output_value(generator->ramp2.speed);
    dt = (cur_time - generator->ramp.last_time)*1e-6;
    generator->type.last_output += dt*speed;
    generator->ramp.last_time = cur_time;
  }

  switch (generator->type.callback.type) {
  case GEN_CALLBACK_NONE :
    break;
  case GEN_CALLBACK_SUP :
    if (generator->type.last_output > generator->type.callback.threshold)
      generator->type.callback.callback_function(generator);
    break;
  case GEN_CALLBACK_INF :
    if (generator->type.last_output < generator->type.callback.threshold)
      generator->type.callback.callback_function(generator);
    break;
  }

  return generator->type.last_output;
}
