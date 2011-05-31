/*
 * bezier_utils.c
 * --------------
 * Copyright : (c) 2011, Xavier Lagorce <Xavier.Lagorce@crans.org>
 * Licence   : BSD3
 *
 * This file is a part of [kro]bot.
 */

#include "bezier_utils.h"

#ifdef BEZIER_UTILS_USE_BERTOS
#include <cpu/power.h>
#endif

float bezier_apply(float params[4], float u) {
  return (params[0] + params[1]*u + params[2]*u*u + params[3]*u*u*u);
}

void bezier_generate(float params[2][4],
                     float p_x, float p_y,
                     float p1_x, float p1_y,
                     float p2_x, float p2_y,
                     float s_x, float s_y) {

  params[0][0] = p_x;
  params[1][0] = p_y;
  params[0][1] = -3*p_x+3*p1_x;
  params[1][1] = -3*p_y+3*p1_y;
  params[0][2] = 3*p_x-6*p1_x+3*p2_x;
  params[1][2] = 3*p_y-6*p1_y+3*p2_y;
  params[0][3] = -p_x+3*p1_x-3*p2_x+s_x;
  params[1][3] = -p_y+3*p1_y-3*p2_y+s_y;
}

void bezier_velocity_profile(float dparams[2][4], float ddparams[2][4],
                             float v_max, float at_max, float ar_max,
                             float v_ini, float v_end,
                             float du, float* v_tab) {
  float vmins[4], cr, cr_p, cr_pp, u, vm, dt, nv, dx, dy, dsu;
  int ind = 0, j, nb_pts, mins[4], im, i;

  for (i=0, u=0.0; u <= 1.0; i++, u+=du) {
    v_tab[i] = v_max;
  }
  nb_pts = i;

  // Looking for curvature radius minima to limit speed at this positions
  mins[ind] = 0;
  vmins[ind] = v_ini;
  ind++;

  cr_pp = fabsf(bezier_cr(0, dparams, ddparams));
  cr_p = fabsf(bezier_cr(du, dparams, ddparams));
  for (i=2, u=2*du; u <= 1.0; i++, u+=du) {
    cr = fabsf(bezier_cr(u, dparams, ddparams));
    if ((cr >= cr_p) && (cr_p < cr_pp)) {
      mins[ind] = i;
      vmins[ind] = sqrtf(ar_max*cr);
      ind++;
    }
    cr_pp = cr_p;
    cr_p = cr;
  }
  mins[ind] = nb_pts-1;
  vmins[ind] = v_end;
  ind++;

#ifdef BEZIER_UTILS_USE_BERTOS
  cpu_relax();
#endif

  // Compute speed limitations
  for (j=0; j < ind; j++) {
    im = mins[j];
    vm = vmins[j];
    if (vm < v_max) {
      v_tab[im] = vm;

      // Profile for preceding velocity
      for (i = im-1; i >= 0; i--) {
        dx = bezier_apply(dparams[0], du*(i+1));
        dy = bezier_apply(dparams[1], du*(i+1));
        dsu = sqrtf(dx*dx + dy*dy);
        dt = (-v_tab[i+1]+sqrtf(v_tab[i+1]*v_tab[i+1]+2*at_max*du*dsu))/at_max;
        nv = v_tab[i+1]+at_max*dt;
        if (nv < v_tab[i]) {
          v_tab[i] = nv;
        } else {
          break;
        }
      }
      // Profile for following sector
      for (i = im+1; i < nb_pts; i++) {
        dx = bezier_apply(dparams[0], du*(i-1));
        dy = bezier_apply(dparams[1], du*(i-1));
        dsu = sqrtf(dx*dx + dy*dy);
        dt = (-v_tab[i-1]+sqrtf(v_tab[i-1]*v_tab[i-1]+2*at_max*du*dsu))/at_max;
        nv = v_tab[i-1]+at_max*dt;
        if (nv < v_tab[i]) {
          v_tab[i] = nv;
        } else {
          break;
        }
      }

#ifdef BEZIER_UTILS_USE_BERTOS
  cpu_relax();
#endif
    } // end if (vm < v_max)
  } // for mins
}

float bezier_cr(float u, float dparams[2][4], float ddparams[2][4]) {

  float dx, dy, ddx, ddy;

  dx = bezier_apply(dparams[0], u);
  dy = bezier_apply(dparams[1], u);
  ddx = bezier_apply(ddparams[0], u);
  ddy = bezier_apply(ddparams[1], u);

  return powf(dx*dx+dy*dy, 1.5) / (dx*ddy - dy*ddx);
}

void bezier_diff(float params[2][4], float dparams[2][4]) {
  int i, j;
  for (i=0; i<=1; i++) {
    for (j=0; j<=2; j++) {
      dparams[i][j] = (j+1)*params[i][j+1];
    }
    dparams[i][3] = 0.;
  }
}
