
#ifndef __WIFI_H
#define __WIFI_H

#include "usart2.h"	   	

#define RESET_IO(x)    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, x)  //PA4����WiFi�ĸ�λ

#define WiFi_printf       u2_printf           //����2���� WiFi
#define WiFi_RxCounter    Usart3_RxCounter    //����2���� WiFi
#define WiFi_RX_BUF       Usart3_RxBuff       //����2���� WiFi
#define WiFi_RXBUFF_SIZE  USART3_RXBUFF_SIZE  //����2���� WiFi

#define SSID   "iQOO Neo5"                          //·����SSID����
#define PASS   "00000000"                    //·��������

extern int ServerPort;
extern char ServerIP[128];                    //��ŷ�����IP��������

void WiFi_ResetIO_Init(void);
char WiFi_SendCmd(char *cmd, int timeout);
char WiFi_Reset(int timeout);
char WiFi_JoinAP(int timeout);
char WiFi_Connect_Server(int timeout);
char WiFi_Smartconfig(int timeout);
char WiFi_WaitAP(int timeout);
char WiFi_Connect_IoTServer(void);


#endif


