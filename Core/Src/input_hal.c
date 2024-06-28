#include "main.h"
#include "stdbool.h"
#include "Delay.h"
#include <stdint.h>

bool Rising_Edge_Detect(bool* State, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint16_t delay_ms)
{
    if(!HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))
    {
        *State = 0;
    }
    if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) && *State == 0)
    {
        Delay_ms(delay_ms);
        if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))
        {
            *State = 1;
            return 1;
        }
    }
    return 0;
}

bool Falling_Edge_Detect(bool* State, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint16_t delay_ms)
{
	if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))
	{
		*State = 1;
	}
	if(!HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) && *State == 1)
	{
		Delay_ms(delay_ms);
		if(!HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))
		{
			*State = 0;
			return 1;
		}
	}
    return 0;
}

int8_t Complementary_input(bool* state, bool* stateN, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint16_t GPIO_PinN)
{
    if(Falling_Edge_Detect(state, GPIOx, GPIO_Pin, 0))
    {
        if(!HAL_GPIO_ReadPin(GPIOx, GPIO_PinN))
        {
            return 1;
        }
    }
    if(Falling_Edge_Detect(stateN, GPIOx, GPIO_PinN, 0))
    {
        if(!HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))
        {
            return -1;
        }
    } 
    return 0;
}

bool Rising_Edge_Detect_Soft(bool* State, bool input)
{
    if(!input)
    {
        *State = 0;
    }
    if(input && *State == 0)
    {
        
            *State = 1;
            return 1;
        
    }
    return 0;
}

bool Falling_Edge_Detect_Soft(bool* State, bool input)
{
	if(input)
	{
		*State = 1;
	}
	if(!input && *State == 1)
	{
			*State = 0;
			return 1;
		
	}
    return 0;
}