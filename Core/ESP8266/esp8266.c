#include "main.h"
//网络设备驱动
#include "esp8266.h"
#include "usart.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "stdarg.h"	


//硬件驱动
//#include "delay.h"
//#include "usart.h"
extern uint8_t USART1_RXbuff;
extern int x;

#define ESP8266_WIFI_INFO		"AT+CWJAP=\"fl\",\"Xk8iQ3654499269\"\r\n"

#define ESP8266_ONENET_INFO		"AT+CIPSTART=\"TCP\",\"183.230.40.39\",6002\r\n"


unsigned char esp8266_buf[128];
unsigned short esp8266_cnt = 0, esp8266_cntPre = 0;

uint8_t aRxBuffer;			//接收中断缓冲
//==========================================================
//	函数名称：	ESP8266_Clear
//
//	函数功能：	清空缓存
//
//	入口参数：	无
//
//	返回参数：	无
//
//	说明：		
//==========================================================
void ESP8266_Clear(void)
{

	memset(esp8266_buf, 0, sizeof(esp8266_buf));
	esp8266_cnt = 0;

}

//==========================================================
//	函数名称：	ESP8266_WaitRecive
//
//	函数功能：	等待接收完成
//
//	入口参数：	无
//
//	返回参数：	REV_OK-接收完成		REV_WAIT-接收超时未完成
//
//	说明：		循环调用检测是否接收完成
//==========================================================
_Bool ESP8266_WaitRecive(void)
{

	if(esp8266_cnt == 0) 							//如果接收计数为0 则说明没有处于接收数据中，所以直接跳出，结束函数
		return REV_WAIT;
		
	if(esp8266_cnt == esp8266_cntPre)				//如果上一次的值和这次相同，则说明接收完毕
	{
		esp8266_cnt = 0;							//清0接收计数
			
		return REV_OK;								//返回接收完成标志
	}
		
	esp8266_cntPre = esp8266_cnt;					//置为相同
	
	return REV_WAIT;								//返回接收未完成标志

}

//==========================================================
//	函数名称：	ESP8266_SendCmd
//
//	函数功能：	发送命令
//
//	入口参数：	cmd：命令
//				res：需要检查的返回指令
//
//	返回参数：	0-成功	1-失败
//
//	说明：		
//==========================================================
_Bool ESP8266_SendCmd(char *cmd, char *res)
{
	
	unsigned char timeOut = 1000;
//	HAL_UART_Transmit(&huart3,(unsigned char *)cmd,strlen((const char *)cmd),0xFFFF);
	Usart_SendString(huart3, (unsigned char *)cmd, strlen((const char *)cmd));
	
	while(timeOut--)
	{
		if(ESP8266_WaitRecive() == REV_OK)							//如果收到数据
		{
			if(strstr((const char *)esp8266_buf, res) != NULL)		//如果检索到关键词
			{
				ESP8266_Clear();									//清空缓存
				
				return 0;
			}
		}
		HAL_Delay(10);
//		delay_ms(10);
	}
	
	return 1;

}

//==========================================================
//	函数名称：	ESP8266_SendData
//
//	函数功能：	发送数据
//
//	入口参数：	data：数据
//				len：长度
//
//	返回参数：	无
//
//	说明：		
//==========================================================
void ESP8266_SendData(unsigned char *data, unsigned short len)
{

	char cmdBuf[32];
	
	ESP8266_Clear();								//清空接收缓存
	sprintf(cmdBuf, "AT+CIPSEND=%d\r\n", len);		//发送命令
	if(!ESP8266_SendCmd(cmdBuf, ">"))				//收到‘>’时可以发送数据
	{
		//HAL_UART_Transmit(&huart3,data,len,0xFFFF);
		Usart_SendString(huart3, data, len);		//发送设备连接请求数据
	}

}

//==========================================================
//	函数名称：	ESP8266_GetIPD
//
//	函数功能：	获取平台返回的数据
//
//	入口参数：	等待的时间(乘以10ms)
//
//	返回参数：	平台返回的原始数据
//
//	说明：		不同网络设备返回的格式不同，需要去调试
//				如ESP8266的返回格式为	"+IPD,x:yyy"	x代表数据长度，yyy是数据内容
//==========================================================
unsigned char *ESP8266_GetIPD(unsigned short timeOut)
{

	char *ptrIPD = NULL;
//	char fake[50] = "+IPD,4:";

	
	do
	{
		if(ESP8266_WaitRecive() == REV_OK)								//如果接收完成
		{
			ptrIPD = strstr((char *)esp8266_buf, "IPD,");				//搜索“IPD”头
//				ptrIPD = fake;
			if(ptrIPD == NULL)											//如果没找到，可能是IPD头的延迟，还是需要等待一会，但不会超过设定的时间
			{
//				UsartPrintf(USART_DEBUG, "\"IPD\" not found\r\n");
			}
			else
			{
				ptrIPD = strchr(ptrIPD, ':');							//找到':'
				if(ptrIPD != NULL)
				{
					ptrIPD++;
					return (unsigned char *)(ptrIPD);
				}
				else
					return NULL;
				
			}
		}
		HAL_Delay(5);
		//delay_ms(5);													//延时等待
	} while(timeOut--);
	
	return NULL;														//超时还未找到，返回空指针

}

//==========================================================
//	函数名称：	ESP8266_Init
//
//	函数功能：	初始化ESP8266
//
//	入口参数：	无
//
//	返回参数：	无
//
//	说明：		
//==========================================================
void ESP8266_Init(void)
{
	ESP8266_Clear();
	
//	UsartPrintf(USART_DEBUG, "0. AT\r\n");
	while(ESP8266_SendCmd("AT\r\n", "OK"))
		HAL_Delay(500);
//		delay_ms(500);
	
//	UsartPrintf(USART_DEBUG, "1. RST\r\n");
	while(ESP8266_SendCmd("AT+CWMODE=1\r\n", "OK"))
		HAL_Delay(500);
//		delay_ms(500);
	
//	UsartPrintf(USART_DEBUG, "2. CWMODE\r\n");
	while(ESP8266_SendCmd("AT+CWDHCP=1,1\r\n", "OK"))
		HAL_Delay(500);
	//	delay_ms(500);
	
//	UsartPrintf(USART_DEBUG, "3. AT+CWDHCP\r\n");       //开启DHCP(获取分配IP),默认保存Flash
//	while(ESP8266_SendCmd("AT+CWDHCP=1,1\r\n", "OK"))  
//		HAL_Delay(500);

	
//	UsartPrintf(USART_DEBUG, "4. CWJAP\r\n");           //链接WIFI
	while(ESP8266_SendCmd(ESP8266_WIFI_INFO, "OK"))
		HAL_Delay(500);
	//	delay_ms(500);
	
//	UsartPrintf(USART_DEBUG, "5. CIPSTART\r\n");        //开启TCP连接
	while(ESP8266_SendCmd(ESP8266_ONENET_INFO, "CONNECT"))
		HAL_Delay(500);
	//delay_ms(500);
	
//	UsartPrintf(USART_DEBUG, "6. ESP8266 Init OK\r\n"); 

}

//==========================================================
//	函数名称：	HAL_UART_RxCpltCallback
//
//	函数功能：	串口2收发中断
//
//	入口参数：	无
//
//	返回参数：	无
//
//	说明：		
//==========================================================
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
////	uint16_t tempt1  /*定义临时变量存放接受的数据*/;
////      if(huart == &huart1)
////    {
////			 tempt1=USART1_RXbuff;
////		 Zigbee_Receive(USART1_RXbuff);	
////        //将接收到的数据发送
////        //HAL_UART_Transmit(huart,(void *)&USART1_RXbuff, sizeof(USART1_RXbuff),0);
////        //重新使能串口接收中断
////        HAL_UART_Receive_IT(huart,(void *)&USART1_RXbuff, 1);
////    }
//    UNUSED(huart);
//	if(huart == &huart3)//esp8266接收云平台数据
//		{
//			if(esp8266_cnt >= sizeof(esp8266_buf))  //溢出判断
//				{
//					esp8266_cnt = 0;
//					memset(esp8266_buf,0x00,sizeof(esp8266_buf));
//					HAL_UART_Transmit(&huart3, (uint8_t *)"接收缓存溢出", 10,0xFFFF); 	      
//				}
//	 else
//		 {
//			 esp8266_buf[esp8266_cnt++] = aRxBuffer;   //接收数据转存	
////		  if(aRxBuffer=='1')  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);	
////        if(aRxBuffer=='0')  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
// 	    }
//	
//	HAL_UART_Receive_IT(&huart3, (uint8_t *)&aRxBuffer, 1);   //再开启接收中断

//		}
//}


//==========================================================
//	函数名称：	UsartPrintf
//
//	函数功能：	串口打印
//
//	入口参数：	无
//
//	返回参数：	无
//
//	说明：		
//========================================================
void UsartPrintf(UART_HandleTypeDef USARTx, char *fmt,...)
{
 
	unsigned char UsartPrintfBuf[296];
	va_list ap;
	unsigned char *pStr = UsartPrintfBuf;
	
	va_start(ap, fmt);
	vsnprintf((char *)UsartPrintfBuf, sizeof(UsartPrintfBuf), fmt, ap);							//格式化
	va_end(ap);
	
	while(*pStr != NULL)
	{
        HAL_UART_Transmit (&USARTx ,(uint8_t *)pStr++,1,HAL_MAX_DELAY );		
	}
 
}

//==========================================================
//	函数名称：	Usart_SendString
//
//	函数功能：	使用串口发送数据
//
//	入口参数：	无
//
//	返回参数：	无
//
//	说明：		
//======================================================
void Usart_SendString(UART_HandleTypeDef USARTx, unsigned char *str, unsigned short len)
{

	unsigned short count = 0;
	
	for(; count < len; count++)
	{
		HAL_UART_Transmit (&USARTx ,(uint8_t *)str++,1,HAL_MAX_DELAY );									//发送数据
	}

}

