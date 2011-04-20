/*
 * differential_drive.c
 * --------------------
 * Copyright : (c) 2011, Xavier Lagorce <Xavier.Lagorce@crans.org>
 * Licence   : BSD3
 *
 * This file is a part of [kro]bot.
 */

#include "differential_drive.h"

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

PROC_DEFINE_STACK(stack_traj_following, KERN_MINSTACKSIZE * 32);

typedef struct {
  uint8_t initialized, enabled;
  float params[2][4], dparams[2][4], ddparams[2][4];
  float du, v_tab[101];
  float goal[2], v_end, theta_end;
  float start[2], theta_ini;
} dd_bezier_traj_t;

typedef struct {
  uint8_t initialized, enabled, running, working;
  float wheel_radius, shaft_width;
  float last_lin_acceleration, last_rot_acceleration;
  float u, v_max, at_max, ar_max;
  command_generator_t left_wheel_speed, right_wheel_speed;
  command_generator_t left_wheel, right_wheel;
  uint8_t current_traj;
  dd_bezier_traj_t trajs[2];
  float Ts, k1, k2, k3;
  robot_state_t ghost_state;
} dd_params_t;

static dd_params_t params;

static void NORETURN traj_following_process(void);

static void NORETURN traj_following_process(void) {
  Timer timer;
  uint8_t next_traj, ui;
  robot_state_t rs;
  float cr, v_lin, v_ratio, v_max, v_rot, dxu, dyu;
  float z1, z2, z3, w1, w2, u1, u2, dt;
  dd_bezier_traj_t *traj;
  int32_t last_time, cur_time;

  // configure timer
  timer_setDelay(&timer, ms_to_ticks(params.Ts * 1000));
  timer_setEvent(&timer);

  // Indicate we are running
  params.running = 1;

  // Init
  params.working = 0;
  next_traj = (params.current_traj + 1) % 2;

  while (1) {
    if (params.enabled == 0) {
      params.running = 0;
      proc_exit();
    } else {
      if (!params.working && params.trajs[next_traj].initialized) {
        LED2_ON();
        params.working = 1;
        params.u = 0;
        ui = 0;
        params.current_traj = next_traj;
        traj = &params.trajs[params.current_traj];
        next_traj = (params.current_traj + 1) % 2;
        params.ghost_state.x = traj->start[0];
        params.ghost_state.y = traj->start[1];
        params.ghost_state.theta = traj->theta_ini;
        traj->enabled = 1;
        last_time = ticks_to_us(timer_clock());
      }
      timer_add(&timer);
      if (params.working) {
        odo_getState(&rs);

        // Stop following the trajectory if we are close enough to our goal
        if (params.u >= 1.0 || ((rs.x-traj->goal[0]) * (rs.x-traj->goal[0]) + (rs.y-traj->goal[1]) * (rs.y-traj->goal[1])) <= (0.01*0.01)) {
        //if (((rs.x-traj->goal[0]) * (rs.x-traj->goal[0]) + (rs.y-traj->goal[1]) * (rs.y-traj->goal[1])) <= (0.01*0.01)) {
          LED2_OFF();
          params.working = 0;
          traj->initialized = 0;
          traj->enabled = 0;
          if (!params.trajs[next_traj].initialized) {
            // We have no other trajectory to follow, let's brake
            dd_set_rotational_speed(0., params.at_max);
            dd_set_linear_speed(0., params.at_max);
          }
        } else {
          // We are following a trajectory, let's do it
          // Compute ghost vehicule parameters
          cr = bezier_cr(params.u, traj->dparams, traj->ddparams);
          for (; ui*traj->du <= params.u; ui++);
          if (ui*traj->du > 1.0)
            ui--;
          if (ui > 0) {
            v_lin = traj->v_tab[ui-1] +
              (traj->v_tab[ui]-traj->v_tab[ui-1]) * (params.u - traj->du*(ui-1))/traj->du;
          } else {
            v_lin = traj->v_tab[0];
          }
          if (isnan(cr) || isinf(cr)) {
            v_ratio = 1.0;
          } else {
            v_ratio = (cr + params.shaft_width/2.0) / (cr - params.shaft_width/2.0);
          }
          v_max = 2/(1+MIN(fabsf(v_ratio), fabsf(1.0/v_ratio)))*v_lin;
          if (cr >= 0) {
            v_rot = v_max * (1 - 1/v_ratio) / params.shaft_width;
          } else {
            v_rot = v_max * (v_ratio - 1) / params.shaft_width;
          }

          // Evolution of the ghost vehicule state
          cur_time = ticks_to_us(timer_clock());
          dt = (cur_time - last_time) * 1e-6;
          dxu = bezier_apply(traj->dparams[0], params.u);
          dyu = bezier_apply(traj->dparams[1], params.u);
          params.u += v_lin/sqrtf(dxu*dxu+dyu*dyu)*dt;
          //if (u >= 1.0) {
          //  u = 1.0;
          //} else {
            params.ghost_state.x += v_lin*cosf(params.ghost_state.theta)*dt;
            params.ghost_state.y += v_lin*sinf(params.ghost_state.theta)*dt;
            params.ghost_state.theta = fmodf(params.ghost_state.theta + v_rot*dt, 2*M_PI);
            if (params.ghost_state.theta > M_PI) {
              params.ghost_state.theta -= 2*M_PI;
            } else if (params.ghost_state.theta <= M_PI) {
              params.ghost_state.theta += 2*M_PI;
            }
            //}

          // Compute command
          z1=(rs.x-params.ghost_state.x)*cosf(params.ghost_state.theta)+(rs.y-params.ghost_state.y)*sinf(params.ghost_state.theta);
          z2=-(rs.x-params.ghost_state.x)*sinf(params.ghost_state.theta)+(rs.y-params.ghost_state.y)*cosf(params.ghost_state.theta);
          z3=tanf(rs.theta-params.ghost_state.theta);

          w1=-params.k1*fabsf(v_lin)*(z1+z2*z3);
          w2=-params.k2*v_lin*z2-params.k3*fabsf(v_lin)*z3;

          u1=(w1+v_lin)/cosf(rs.theta-params.ghost_state.theta);
          u2=w2*powf(cosf(rs.theta-params.ghost_state.theta),2)+v_rot;

          // Apply command
          dd_set_linear_speed(u1, 0.);
          dd_set_rotational_speed(u2, 0.);

          // Keep current time
          last_time = cur_time;
        }
      }
    }
    timer_waitEvent(&timer); // Wait for the remaining of the sample period
  }
}

void dd_start(float wheel_radius, float shaft_width, float max_wheel_speed,
              float v_max, float at_max, float ar_max,
              float k1, float k2, float k3, float Ts) {
  params.wheel_radius = wheel_radius;
  params.shaft_width = shaft_width;
  params.last_lin_acceleration = 0.0;
  params.last_rot_acceleration = 0.0;

  params.ghost_state.x = 0;
  params.ghost_state.y = 0;
  params.ghost_state.theta = 0;

  params.running = 0;
  params.working = 0;
  params.current_traj = 0;
  params.trajs[0].initialized = 0;
  params.trajs[1].initialized = 0;
  params.trajs[0].enabled = 0;
  params.trajs[1].enabled = 0;
  params.v_max = v_max;
  params.at_max = at_max;
  params.ar_max = ar_max;

  params.k1 = k1;
  params.k2 = k2;
  params.k3 = k3;
  params.Ts = Ts;

  tc_new_controller(DD_LINEAR_SPEED_TC);
  tc_new_controller(DD_ROTATIONAL_SPEED_TC);
  new_dd_generator(&params.left_wheel_speed,
                   tc_get_position_generator(DD_LINEAR_SPEED_TC),
                   tc_get_speed_generator(DD_LINEAR_SPEED_TC),
                   tc_get_position_generator(DD_ROTATIONAL_SPEED_TC),
                   tc_get_speed_generator(DD_ROTATIONAL_SPEED_TC),
                   wheel_radius, shaft_width, max_wheel_speed,
                   -1);
  new_dd_generator(&params.right_wheel_speed,
                   tc_get_position_generator(DD_LINEAR_SPEED_TC),
                   tc_get_speed_generator(DD_LINEAR_SPEED_TC),
                   tc_get_position_generator(DD_ROTATIONAL_SPEED_TC),
                   tc_get_speed_generator(DD_ROTATIONAL_SPEED_TC),
                   wheel_radius, shaft_width, max_wheel_speed,
                   1);
  new_ramp2_generator(&params.left_wheel, 0.0, &params.left_wheel_speed);
  new_ramp2_generator(&params.right_wheel, 0.0, &params.right_wheel_speed);

  start_generator(&params.left_wheel_speed);
  start_generator(&params.right_wheel_speed);
  start_generator(&params.left_wheel);
  start_generator(&params.right_wheel);

  params.initialized = 1;
  params.enabled = 1;

  proc_new(traj_following_process, NULL, sizeof(stack_traj_following), stack_traj_following);
}

void dd_pause(void) {
  if (params.initialized) {
    dd_set_linear_speed(0.0, params.last_lin_acceleration);
    dd_set_rotational_speed(0.0, params.last_rot_acceleration);
    params.enabled = 0;
  }
}

void dd_resume(void) {
  if (params.initialized && params.enabled) {
    params.enabled = 1;
  }
}

void dd_stop(void) {
  if (params.initialized) {
    pause_generator(&params.left_wheel);
    pause_generator(&params.right_wheel);
    pause_generator(&params.left_wheel_speed);
    pause_generator(&params.right_wheel_speed);
    tc_delete_controller(DD_LINEAR_SPEED_TC);
    tc_delete_controller(DD_ROTATIONAL_SPEED_TC);
    params.enabled = 0;
    params.initialized = 0;
  }
}

command_generator_t* dd_get_left_wheel_generator(void) {
  return &params.left_wheel;
}

command_generator_t* dd_get_right_wheel_generator(void) {
  return &params.right_wheel;
}

void dd_move(float distance, float speed, float acceleration) {
  if (params.enabled) {
    params.last_lin_acceleration = acceleration;
    tc_goto(DD_LINEAR_SPEED_TC, distance, speed, params.last_lin_acceleration);
  }
}

void dd_turn(float angle, float speed, float acceleration) {
  if (params.enabled) {
    params.last_rot_acceleration = acceleration;
    tc_goto(DD_ROTATIONAL_SPEED_TC, angle, speed, params.last_rot_acceleration);
  }
}

void dd_set_linear_speed(float speed, float acceleration) {
  if (params.enabled) {
    if (acceleration != 0.) {
      params.last_lin_acceleration = acceleration;
      tc_goto_speed(DD_LINEAR_SPEED_TC, speed, params.last_lin_acceleration);
    } else {
      tc_set_speed(DD_LINEAR_SPEED_TC, speed);
    }
  }
}

void dd_set_rotational_speed(float speed, float acceleration) {
  if (params.enabled) {
    if (acceleration != 0.) {
      params.last_rot_acceleration = acceleration;
      tc_goto_speed(DD_ROTATIONAL_SPEED_TC, speed, params.last_rot_acceleration);
    } else {
      tc_set_speed(DD_ROTATIONAL_SPEED_TC, speed);
    }
  }
}

uint8_t dd_add_bezier(float x_end, float y_end, float d1, float d2, float end_angle, float end_speed) {
  uint8_t t_ind;
  robot_state_t rs;
  dd_bezier_traj_t *traj;
  float v_ini, x_ini, y_ini, theta_ini;

  t_ind = (params.current_traj + 1) % 2;

  if (params.trajs[t_ind].initialized == 1) {
    return DD_TRAJECTORY_ALREADY_USED;
  } else {
    traj = &params.trajs[t_ind];
    // New trajectory is not enabled
    traj->enabled = 0;

    // Get starting parameters
    if (params.trajs[params.current_traj].enabled) {
      v_ini = params.trajs[params.current_traj].v_end;
      x_ini = params.trajs[params.current_traj].goal[0];
      y_ini = params.trajs[params.current_traj].goal[1];
      theta_ini = params.trajs[params.current_traj].theta_end;
    } else {
      odo_getState(&rs);
      x_ini = rs.x;
      y_ini = rs.y;
      theta_ini = rs.theta;
      v_ini = 0.01; // non null low velocity to allow the system to start
    }

    // Compute Bezier Spline parameters
    bezier_generate(traj->params,
                    x_ini, y_ini,
                    x_ini + d1*cosf(theta_ini), y_ini + d1*sinf(theta_ini),
                    x_end - d2*cosf(end_angle), y_end - d2*sinf(end_angle),
                    x_end, y_end);
    traj->goal[0] = x_end;
    traj->goal[1] = y_end;
    traj->v_end = end_speed;
    traj->theta_end = end_angle;
    traj->start[0] = x_ini;
    traj->start[1] = y_ini;
    traj->theta_ini = theta_ini;
    // Differentiate parameters
    bezier_diff(traj->params, traj->dparams);
    bezier_diff(traj->dparams, traj->ddparams);
    // Compute velocity profile
    bezier_velocity_profile(traj->dparams, traj->ddparams,
                            params.v_max, params.at_max, params.ar_max,
                            v_ini, end_speed,
                            0.01, traj->v_tab);
    traj->du = 0.01;

    params.trajs[t_ind].initialized = 1;
    return DD_NO_ERROR;
  }
}

uint8_t dd_get_ghost_state(robot_state_t *state, float *u) {
  state->x = params.ghost_state.x;
  state->y = params.ghost_state.y;
  state->theta = params.ghost_state.theta;
  *u = params.u;

  if (params.working == 1)
    return DD_GHOST_MOVING;
  else
    return DD_GHOST_STOPPED;
}
