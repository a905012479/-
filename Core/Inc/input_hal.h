#ifndef __Input
#define __Input

bool Rising_Edge_Detect(bool* State, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint16_t delay_ms);
bool Falling_Edge_Detect(bool* State, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint16_t delay_ms);
int8_t Complementary_input(bool* state, bool* stateN, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint16_t GPIO_PinN);
bool Rising_Edge_Detect_Soft(bool* State, bool input);
bool Falling_Edge_Detect_Soft(bool* State, bool input);


#endif
