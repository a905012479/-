#include "stm32f10x.h"
#include "stdbool.h"

void Reverse(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    GPIO_WriteBit(GPIOx, GPIO_Pin, (~GPIO_ReadOutputDataBit(GPIOx, GPIO_Pin)) & 0x01);
}
