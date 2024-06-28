#ifndef __I2C_HAL_H
#define __I2C_HAL_H

#include "main.h"
// #include "gpio.h"
#include "stdbool.h"
#define SDA_PIN GPIO_PIN_9
#define SCL_PIN GPIO_PIN_8
#define I2C_PORT GPIOB

void I2C_Init_C();
void I2C_Start(void);
void I2C_Stop(void);
void I2C_SendByte(uint8_t Data);
uint8_t I2C_ReceiveByte(void);
void I2C_ACK(bool State);
bool I2C_WaitACK(void);

void MEM_Read(unsigned char Device_Addr,unsigned char *ucBuf, unsigned char ucAddr, unsigned char ucNum);
void MEM_Write(unsigned char Device_Addr, unsigned char *ucBuf, unsigned char ucAddr, unsigned char ucNum);
void MCP4017_Read(unsigned char *ucBuf);
void MCP4017_Write(unsigned char ucBuf);


#endif
