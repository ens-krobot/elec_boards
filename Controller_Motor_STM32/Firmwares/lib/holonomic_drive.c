/*
 * holonomic_drive.c
 * --------------------
 * Copyright : (c) 2011, Xavier Lagorce <Xavier.Lagorce@crans.org>
 * Licence   : BSD3
 *
 * This file is a part of [kro]bot.
 */

#include "holonomic_drive.h"

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

typedef struct {
  uint8_t initialized, enabled, running, working;
  uint8_t enable_transform;
  float wheel_radius, drive_radius;
  float v_lin_max, v_rot_max, acc_lin_max, acc_rot_max;
  command_generator_t f_wheel_speed, br_wheel_speed, bl_wheel_speed;
  command_generator_t f_wheel, br_wheel, bl_wheel;
  float Ts;
} hd_params_t;

static hd_params_t params;

void hd_start(uint8_t enable_transform,
              float wheel_radius, float drive_radius,
              float max_wheel_speed,
              float Ts) {
  params.enable_transform = enable_transform;
  params.wheel_radius = wheel_radius;
  params.drive_radius = drive_radius;

  params.v_lin_max = 0.3;
  params.v_rot_max = M_PI/4.;
  params.acc_lin_max = 0.5;
  params.acc_rot_max = M_PI/4;

  params.running = 0;
  params.working = 0;
  params.Ts = Ts;

  tc_new_controller(HD_LINEAR_SPEED_X_TC);
  tc_new_controller(HD_LINEAR_SPEED_Y_TC);
  tc_new_controller(HD_ROTATIONAL_SPEED_TC);
  new_hd_generator(&params.f_wheel_speed,
                   tc_get_position_generator(HD_LINEAR_SPEED_X_TC),
                   tc_get_speed_generator(HD_LINEAR_SPEED_X_TC),
                   tc_get_position_generator(HD_LINEAR_SPEED_Y_TC),
                   tc_get_speed_generator(HD_LINEAR_SPEED_Y_TC),
                   tc_get_position_generator(HD_ROTATIONAL_SPEED_TC),
                   tc_get_speed_generator(HD_ROTATIONAL_SPEED_TC),
                   wheel_radius, drive_radius, max_wheel_speed,
                   params.enable_transform, 1);
  new_hd_generator(&params.br_wheel_speed,
                   tc_get_position_generator(HD_LINEAR_SPEED_X_TC),
                   tc_get_speed_generator(HD_LINEAR_SPEED_X_TC),
                   tc_get_position_generator(HD_LINEAR_SPEED_Y_TC),
                   tc_get_speed_generator(HD_LINEAR_SPEED_Y_TC),
                   tc_get_position_generator(HD_ROTATIONAL_SPEED_TC),
                   tc_get_speed_generator(HD_ROTATIONAL_SPEED_TC),
                   wheel_radius, drive_radius, max_wheel_speed,
                   params.enable_transform, 2);
  new_hd_generator(&params.bl_wheel_speed,
                   tc_get_position_generator(HD_LINEAR_SPEED_X_TC),
                   tc_get_speed_generator(HD_LINEAR_SPEED_X_TC),
                   tc_get_position_generator(HD_LINEAR_SPEED_Y_TC),
                   tc_get_speed_generator(HD_LINEAR_SPEED_Y_TC),
                   tc_get_position_generator(HD_ROTATIONAL_SPEED_TC),
                   tc_get_speed_generator(HD_ROTATIONAL_SPEED_TC),
                   wheel_radius, drive_radius, max_wheel_speed,
                   params.enable_transform, 3);
  new_ramp2_generator(&params.f_wheel, 0.0, &params.f_wheel_speed);
  new_ramp2_generator(&params.br_wheel, 0.0, &params.br_wheel_speed);
  new_ramp2_generator(&params.bl_wheel, 0.0, &params.bl_wheel_speed);

  start_generator(&params.f_wheel_speed);
  start_generator(&params.br_wheel_speed);
  start_generator(&params.bl_wheel_speed);
  start_generator(&params.f_wheel);
  start_generator(&params.br_wheel);
  start_generator(&params.bl_wheel);

  params.initialized = 1;
  params.enabled = 1;
}

void hd_pause(void) {
  if (params.initialized) {
    hd_set_linear_speed_X(0.0, params.acc_lin_max);
    hd_set_linear_speed_Y(0.0, params.acc_lin_max);
    hd_set_rotational_speed(0.0, params.acc_rot_max);
    params.enabled = 0;
  }
}

void hd_resume(void) {
  if (params.initialized && params.enabled) {
    params.enabled = 1;
  }
}

void hd_stop(void) {
  if (params.initialized) {
    pause_generator(&params.f_wheel);
    pause_generator(&params.br_wheel);
    pause_generator(&params.bl_wheel);
    pause_generator(&params.f_wheel_speed);
    pause_generator(&params.br_wheel_speed);
    pause_generator(&params.bl_wheel_speed);
    tc_delete_controller(HD_LINEAR_SPEED_X_TC);
    tc_delete_controller(HD_LINEAR_SPEED_Y_TC);
    tc_delete_controller(HD_ROTATIONAL_SPEED_TC);
    params.enabled = 0;
    params.initialized = 0;
  }
}

command_generator_t* hd_get_front_wheel_generator(void) {
  return &params.f_wheel;
}

command_generator_t* hd_get_back_right_wheel_generator(void) {
  return &params.br_wheel;
}

command_generator_t* hd_get_back_left_wheel_generator(void) {
  return &params.bl_wheel;
}

void hd_move_X(float distance, float speed, float acceleration) {
  if (params.enabled) {
    tc_move(HD_LINEAR_SPEED_X_TC, distance, speed, acceleration);
  }
}

void hd_move_Y(float distance, float speed, float acceleration) {
  if (params.enabled) {
    tc_move(HD_LINEAR_SPEED_Y_TC, distance, speed, acceleration);
  }
}

void hd_turn(float angle, float speed, float acceleration) {
  if (params.enabled) {
    tc_move(HD_ROTATIONAL_SPEED_TC, angle, speed, acceleration);
  }
}

void hd_move2D(float dx, float dy) {
  if (params.enabled) {
    float adx = dx >= 0. ? dx: -dx;
    float ady = dy >= 0. ? dy: -dy;
    float d = sqrt(adx*adx+ady*ady);
    if (d > 0.) {
      float vx, vy, ax, ay;
      vx = params.v_lin_max*adx/d;
      vy = params.v_lin_max*ady/d;
      ax = params.acc_lin_max*adx/d;
      ay = params.acc_lin_max*ady/d;
      hd_move_X(dx, vx, ax);
      hd_move_Y(dy, vy, ay);
    }
  }
}

void hd_move_to(float x, float y, float theta) {
  if (params.enabled) {
    // Compute required movement
    robot_state_t odometry;
    HolOdo_getState(&odometry);
    float dx = x - odometry.x;
    float dy = y - odometry.y;
    float dtheta = theta - odometry.theta;
    if (dtheta > M_PI) {
      dtheta -= 2*M_PI;
    }
    if (dtheta < -M_PI) {
      dtheta += 2*M_PI;
    }
    hd_move2D(dx, dy);
    hd_turn(dtheta, params.v_rot_max, params.acc_rot_max);
  }
}

void hd_set_linear_speed_X(float speed, float acceleration) {
  if (params.enabled) {
    if (acceleration != 0.) {
      tc_goto_speed(HD_LINEAR_SPEED_X_TC, speed, acceleration);
    } else {
      tc_set_speed(HD_LINEAR_SPEED_X_TC, speed);
    }
  }
}

void hd_set_linear_speed_Y(float speed, float acceleration) {
  if (params.enabled) {
    if (acceleration != 0.) {
      tc_goto_speed(HD_LINEAR_SPEED_Y_TC, speed, acceleration);
    } else {
      tc_set_speed(HD_LINEAR_SPEED_Y_TC, speed);
    }
  }
}

void hd_set_rotational_speed(float speed, float acceleration) {
  if (params.enabled) {
    if (acceleration != 0.) {
      tc_goto_speed(HD_ROTATIONAL_SPEED_TC, speed, acceleration);
    } else {
      tc_set_speed(HD_ROTATIONAL_SPEED_TC, speed);
    }
  }
}

void hd_adjust_limits(float v_lin_max, float v_rot_max, float acc_lin_max, float acc_rot_max) {
  params.v_lin_max = v_lin_max;
  params.v_rot_max = v_rot_max;
  params.acc_lin_max = acc_lin_max;
  params.acc_rot_max = acc_rot_max;
}
