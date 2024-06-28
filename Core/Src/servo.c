#include "servo.h"

void Servo_Angle(double angle, double Biggest_Angle, TIM_HandleTypeDef *__HANDLE__, uint32_t __CHANNEL__)
{
//    __HAL_TIM_SET_COMPARE(__HANDLE__, __CHANNEL__, angle / Biggest_Angle * 0.4 * 20000 + 2000); //200Hz
	__HAL_TIM_SET_COMPARE(__HANDLE__, __CHANNEL__, angle / Biggest_Angle * 0.5 * 20000 + 2500); //250Hz
}
