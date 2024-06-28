#ifndef __Solar
#define __Solar

#include "stm32g4xx_hal.h"

double Solar_Delta(void);
double Solar_Angle_Alpha(float latitude);
double Solar_Angle_A(float latitude);
void Get_Sun(double latitude, double longitude, double *Alt, double *Azi);

#endif
