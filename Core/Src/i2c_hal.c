#include "i2c_hal.h"
#include "delay.h"


void I2C_Init_C()
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pin = SCL_PIN | SDA_PIN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(I2C_PORT, &GPIO_InitStruct);
}

void SDA_INPUT_MODE(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pin = SDA_PIN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(I2C_PORT, &GPIO_InitStruct);
	delay1(10);
}

void SDA_OUTPUT_MODE(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pin = SDA_PIN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(I2C_PORT, &GPIO_InitStruct);
	delay1(10);
}

void I2C_SDA_OUT(bool State)
{
    HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, State);
    delay1(10);
}

void I2C_SCL_OUT(bool State)
{
	HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, State);

    delay1(10);
}

bool I2C_SDA_INPUT(void)
{

    int Val;
    Val = HAL_GPIO_ReadPin(I2C_PORT, SDA_PIN);
    delay1(10);
    return Val;
}

void I2C_Start(void)
{
    I2C_SDA_OUT(1);
    I2C_SCL_OUT(1);
    I2C_SDA_OUT(0);
    I2C_SCL_OUT(0);
}

void I2C_Stop(void)
{
    I2C_SCL_OUT(0);
    I2C_SDA_OUT(0);
    I2C_SCL_OUT(1);
    I2C_SDA_OUT(1);
}

void I2C_SendByte(uint8_t Data)
{
    I2C_SCL_OUT(0);
    for (int i = 0; i < 8; i++)
    {
        I2C_SDA_OUT(Data & 0x80);
        I2C_SCL_OUT(1);
        I2C_SCL_OUT(0);
        Data <<= 1;
    }
}

uint8_t I2C_ReceiveByte(void)
{
    uint8_t Val = 0;
    I2C_SDA_OUT(1);
    for (int i = 0; i < 8; i++)
    {
        I2C_SCL_OUT(1);
        Val += I2C_SDA_INPUT();
        Val <<= 1;
        I2C_SCL_OUT(0);
    }
    return Val >>= 1;
}

void I2C_ACK(bool State)
{
	I2C_SCL_OUT(0);
    I2C_SDA_OUT(State);
    I2C_SCL_OUT(1);
    I2C_SDA_OUT(1);
}

bool I2C_WaitACK(void)
{
    bool Val;
    I2C_SDA_OUT(1);
    I2C_SCL_OUT(1);
    Val = I2C_SDA_INPUT();
    I2C_SCL_OUT(0);
    return Val;
}

void MEM_Read(unsigned char Device_Addr, unsigned char *ucBuf, unsigned char ucAddr, unsigned char ucNum)
{
    I2C_Start();
    I2C_SendByte(Device_Addr);
    I2C_WaitACK();

    I2C_SendByte(ucAddr);
    I2C_WaitACK();

    I2C_Start();
    I2C_SendByte(Device_Addr | 0x01);
    I2C_WaitACK();

    while (ucNum--)
    {

        *ucBuf++ = I2C_ReceiveByte();
        if (ucNum)
        {
            I2C_ACK(0);
        }
        else
        {
            I2C_ACK(1);
        }
    }

    I2C_Stop();
}

void MEM_Write(unsigned char Device_Addr, unsigned char *ucBuf, unsigned char ucAddr, unsigned char ucNum)
{
    I2C_Start();
    I2C_SendByte(Device_Addr);
    I2C_WaitACK();

    I2C_SendByte(ucAddr);
    I2C_WaitACK();
    I2C_SendByte(*ucBuf);
    I2C_WaitACK();
    I2C_Stop();
    delay1(500);
}

void MCP4017_Write(unsigned char ucBuf)
{
    I2C_Start();
    I2C_SendByte(0x5e);
    I2C_WaitACK();

    I2C_SendByte(ucBuf);
    I2C_WaitACK();
    I2C_Stop();
}

void MCP4017_Read(unsigned char *ucBuf)
{

    I2C_Start();
    I2C_SendByte(0x5f);
    I2C_WaitACK();

    *ucBuf = I2C_ReceiveByte();
    I2C_ACK(1);
    I2C_Stop();
}
