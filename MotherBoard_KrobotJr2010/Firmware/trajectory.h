/*
 * Trajectory generation
 * Xavier Lagorce
 */

#ifndef HEADER__TRAJECTORY
#define HEADER__TRAJECTORY

#include "ch.h"
#include <math.h>
#include "speed_control.h"

// Mecanical characteristics
#define SRADIUS 150 // Structural radius (mm)
#define WRADIUS  25 // Wheel radius (mm)

#define SQRT3_2 87//SQRT3_2 0.866025 // sqrt(3)/2 ... * 100

void setScrew(int16_t ptX, int16_t ptY, int16_t vX, int16_t vY, int16_t omega);

void turn(int16_t omega);

#endif
