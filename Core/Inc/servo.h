#ifndef __Servo
#define __Servo

#include "stm32g4xx_hal.h"


void Servo_Angle(double angle, double Biggest_Angle, TIM_HandleTypeDef* __HANDLE__, uint32_t __CHANNEL__);

#endif
