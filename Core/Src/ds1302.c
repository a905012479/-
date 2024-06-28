#include "ds1302.h"
#include "stm32g4xx_hal.h"
#include "delay.h"

struct TIMEData TimeData;
uint8_t read_time[7];

void ds1302_gpio_init()                                           // CE,SCLK端口初始化
{
    GPIO_InitTypeDef GPIO_InitStructure;
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStructure.Pin = GPIO_PIN_13;                         // PB.13  CE
    GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;                // 推挽输出
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);                    // 初始化GPIOB.13
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

    GPIO_InitStructure.Pin = GPIO_PIN_14;                         // PB.14  SCLK
    GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;                // 推挽输出
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);                    // 初始化GPIOB.14
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}

void ds1032_DATAOUT_init()                                        // 配置双向I/O端口为输出态
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.Pin = GPIO_PIN_15;                         // PB.15  DATA
    GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);                    // 初始化GPIOB.15
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
}

void ds1032_DATAINPUT_init()                                      // 配置双向I/O端口为输入态
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.Pin = GPIO_PIN_15;                         // PB.15 DATA
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);                    // 初始化GPIOB.15
}

void ds1302_write_onebyte(uint8_t data)                           // 向DS1302发送一字节数据
{
    ds1032_DATAOUT_init();
    uint8_t count = 0;
    SCLK_L;
    for (count = 0; count < 8; count++)
    {
        SCLK_L;
        if (data & 0x01)
        {
            DATA_H;
        }
        else
        {
            DATA_L;
        }                                                           // 先准备好数据再发送
        SCLK_H;                                                     // 拉高时钟线，发送数据
        data >>= 1;
    }
}

void ds1302_wirte_rig(uint8_t address, uint8_t data)                // 向指定寄存器地址发送数据
{
    uint8_t temp1 = address;
    uint8_t temp2 = data;
    CE_L;
    SCLK_L;
    Delay_us(1);
    CE_H;
    Delay_us(2);
    ds1302_write_onebyte(temp1);
    ds1302_write_onebyte(temp2);
    CE_L;
    SCLK_L;
    Delay_us(2);
}

uint8_t ds1302_read_rig(uint8_t address)                // 从指定地址读取一字节数据
{
    uint8_t temp3 = address;
    uint8_t count = 0;
    uint8_t return_data = 0x00;
    CE_L;
    SCLK_L;
    Delay_us(3);
    CE_H;
    Delay_us(3);
    ds1302_write_onebyte(temp3);
    ds1032_DATAINPUT_init();                            // 配置I/O口为输入
    Delay_us(2);
    for (count = 0; count < 8; count++)
    {
        Delay_us(2);                                    // 使电平持续一段时间
        return_data >>= 1;
        SCLK_H;
        Delay_us(4);                                    // 使高电平持续一段时间
        SCLK_L;
        Delay_us(14);                                   // 延时14us后再去读取电压，更加准确
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15))
        {
            return_data = return_data | 0x80;
        }
    }
    Delay_us(2);
    CE_L;
    DATA_L;
    return return_data;
}

void ds1032_init()
{
    ds1302_wirte_rig(0x8e, 0x00);                        // 关闭写保护
    ds1302_wirte_rig(0x80, 0x37);                        // seconds37秒
    ds1302_wirte_rig(0x82, 0x08);                        // minutes58分
    ds1302_wirte_rig(0x84, 0x16);                        // hours23时
    ds1302_wirte_rig(0x86, 0x16);                        // date30日
    ds1302_wirte_rig(0x88, 0x12);                        // months9月
    ds1302_wirte_rig(0x8a, 0x06);                        // days星期日
    ds1302_wirte_rig(0x8c, 0x23);                        // year2020年
    ds1302_wirte_rig(0x8e, 0x80);                        // 关闭写保护
}

void ds1032_read_time()
{
    read_time[0] = ds1302_read_rig(0x81);                // 读秒
    read_time[1] = ds1302_read_rig(0x83);                // 读分
    read_time[2] = ds1302_read_rig(0x85);                // 读时
    read_time[3] = ds1302_read_rig(0x87);                // 读日
    read_time[4] = ds1302_read_rig(0x89);                // 读月
    read_time[5] = ds1302_read_rig(0x8B);                // 读星期
    read_time[6] = ds1302_read_rig(0x8D);                // 读年
}

void ds1032_read_realTime()
{
    ds1032_read_time();                                  // BCD码转换为10进制
    TimeData.second = (read_time[0] >> 4) * 10 + (read_time[0] & 0x0f);
    TimeData.minute = ((read_time[1] >> 4) & (0x07)) * 10 + (read_time[1] & 0x0f);
    TimeData.hour = (read_time[2] >> 4) * 10 + (read_time[2] & 0x0f);
    TimeData.day = (read_time[3] >> 4) * 10 + (read_time[3] & 0x0f);
    TimeData.month = (read_time[4] >> 4) * 10 + (read_time[4] & 0x0f);
    TimeData.week = read_time[5];
    TimeData.year = (read_time[6] >> 4) * 10 + (read_time[6] & 0x0f) + 2000;
}