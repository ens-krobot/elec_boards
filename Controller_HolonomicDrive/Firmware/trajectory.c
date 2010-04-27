/*
 * Trajectory generation
 * Xavier Lagorce
 */

#include "trajectory.h"

void setScrew(int16_t ptX, int16_t ptY, int16_t vX, int16_t vY, int16_t omega) {

  int32_t w1, w2, w3, om, tmp;

  // Real formulas, unusable because of floating point arithmetic...
  /*om = (((float)omega)/180.0*M_PI);

  w1 = (int32_t)((-vX - (ptY-SRADIUS)*om)/WRADIUS*180.0/M_PI);
  w2 = (int32_t)((vX/2.0+(ptY+SRADIUS/2.0)*om/2.0-(vY-(ptX+SQRT3_2*SRADIUS)*om)*SQRT3_2)/WRADIUS*180.0/M_PI);
  w3 = (int32_t)((vX/2.0+(ptY+SRADIUS/2.0)*om/2.0+(vY-(ptX-SQRT3_2*SRADIUS)*om)*SQRT3_2)/WRADIUS*180.0/M_PI);*/

  om = (((int32_t)omega)*314);

  w1 = (int32_t)((-(int32_t)vX*18000 - ((int32_t)ptY-SRADIUS)*om)/(314*WRADIUS));
  tmp = ((int32_t)vX*18000+((int32_t)ptY+SRADIUS/2)*om)/2;
  w2 = (int32_t)((tmp-((int32_t)vY*18000-((int32_t)ptX*10+SQRT3_2*SRADIUS/10)*om/10)*SQRT3_2/100)/(WRADIUS*314));
  w3 = (int32_t)((tmp+((int32_t)vY*18000-((int32_t)ptX*10-SQRT3_2*SRADIUS/10)*om/10)*SQRT3_2/100)/(WRADIUS*314));

  sc_setRefSpeed(MOTOR1, w1);
  sc_setRefSpeed(MOTOR2, w2);
  sc_setRefSpeed(MOTOR3, w3);
}

void turn(int16_t omega) {
  setScrew(0, 0, 0, 0, omega);
}
