/*
 * bezier_utils.h
 * --------------
 * Copyright : (c) 2011, Xavier Lagorce <Xavier.Lagorce@crans.org>
 * Licence   : BSD3
 *
 * This file is a part of [kro]bot.
 */

#ifndef __BEZIER_UTILS_H
#define __BEZIER_UTILS_H

#define BEZIER_UTILS_USE_BERTOS

#include <math.h>
#include <stdint.h>

/*
 * Apply a polynomial form of a Bezier Spline to a value of the parameter
 */
float bezier_apply(float params[4], float u);

/*
 * Computes the parameters of the polynomial from of a Bezier Spline
 */
void bezier_generate(float params[2][4],
                     float p_x, float p_y,
                     float p1_x, float p1_y,
                     float p2_x, float p2_y,
                     float s_x, float s_y);

/*
 * Computes the velocity profile along a Bezier Spline and according to physical
 * contraints
 */
void bezier_velocity_profile(float dparams[2][4], float ddparams[2][4],
                             float v_max, float at_max, float ar_max,
                             float v_ini, float v_end,
                             float du, float* v_tab);

/*
 * Computes the curvature radius of a Bezier Spline for a given value of the
 * parameter
 */
float bezier_cr(float u, float dparams[2][4], float ddparams[2][4]);

/*
 * Differentiates the polynomial form of a Bezier Spline
 */
void bezier_diff(float params[2][4], float dparams[2][4]);

#endif /* __BEZIER_UTILS_H */
