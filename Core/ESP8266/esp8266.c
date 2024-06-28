#include "main.h"
//�����豸����
#include "esp8266.h"
#include "usart.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "stdarg.h"	


//Ӳ������
//#include "delay.h"
//#include "usart.h"
extern uint8_t USART1_RXbuff;
extern int x;

#define ESP8266_WIFI_INFO		"AT+CWJAP=\"fl\",\"Xk8iQ3654499269\"\r\n"

#define ESP8266_ONENET_INFO		"AT+CIPSTART=\"TCP\",\"183.230.40.39\",6002\r\n"


unsigned char esp8266_buf[128];
unsigned short esp8266_cnt = 0, esp8266_cntPre = 0;

uint8_t aRxBuffer;			//�����жϻ���
//==========================================================
//	�������ƣ�	ESP8266_Clear
//
//	�������ܣ�	��ջ���
//
//	��ڲ�����	��
//
//	���ز�����	��
//
//	˵����		
//==========================================================
void ESP8266_Clear(void)
{

	memset(esp8266_buf, 0, sizeof(esp8266_buf));
	esp8266_cnt = 0;

}

//==========================================================
//	�������ƣ�	ESP8266_WaitRecive
//
//	�������ܣ�	�ȴ��������
//
//	��ڲ�����	��
//
//	���ز�����	REV_OK-�������		REV_WAIT-���ճ�ʱδ���
//
//	˵����		ѭ�����ü���Ƿ�������
//==========================================================
_Bool ESP8266_WaitRecive(void)
{

	if(esp8266_cnt == 0) 							//������ռ���Ϊ0 ��˵��û�д��ڽ��������У�����ֱ����������������
		return REV_WAIT;
		
	if(esp8266_cnt == esp8266_cntPre)				//�����һ�ε�ֵ�������ͬ����˵���������
	{
		esp8266_cnt = 0;							//��0���ռ���
			
		return REV_OK;								//���ؽ�����ɱ�־
	}
		
	esp8266_cntPre = esp8266_cnt;					//��Ϊ��ͬ
	
	return REV_WAIT;								//���ؽ���δ��ɱ�־

}

//==========================================================
//	�������ƣ�	ESP8266_SendCmd
//
//	�������ܣ�	��������
//
//	��ڲ�����	cmd������
//				res����Ҫ���ķ���ָ��
//
//	���ز�����	0-�ɹ�	1-ʧ��
//
//	˵����		
//==========================================================
_Bool ESP8266_SendCmd(char *cmd, char *res)
{
	
	unsigned char timeOut = 1000;
//	HAL_UART_Transmit(&huart3,(unsigned char *)cmd,strlen((const char *)cmd),0xFFFF);
	Usart_SendString(huart3, (unsigned char *)cmd, strlen((const char *)cmd));
	
	while(timeOut--)
	{
		if(ESP8266_WaitRecive() == REV_OK)							//����յ�����
		{
			if(strstr((const char *)esp8266_buf, res) != NULL)		//����������ؼ���
			{
				ESP8266_Clear();									//��ջ���
				
				return 0;
			}
		}
		HAL_Delay(10);
//		delay_ms(10);
	}
	
	return 1;

}

//==========================================================
//	�������ƣ�	ESP8266_SendData
//
//	�������ܣ�	��������
//
//	��ڲ�����	data������
//				len������
//
//	���ز�����	��
//
//	˵����		
//==========================================================
void ESP8266_SendData(unsigned char *data, unsigned short len)
{

	char cmdBuf[32];
	
	ESP8266_Clear();								//��ս��ջ���
	sprintf(cmdBuf, "AT+CIPSEND=%d\r\n", len);		//��������
	if(!ESP8266_SendCmd(cmdBuf, ">"))				//�յ���>��ʱ���Է�������
	{
		//HAL_UART_Transmit(&huart3,data,len,0xFFFF);
		Usart_SendString(huart3, data, len);		//�����豸������������
	}

}

//==========================================================
//	�������ƣ�	ESP8266_GetIPD
//
//	�������ܣ�	��ȡƽ̨���ص�����
//
//	��ڲ�����	�ȴ���ʱ��(����10ms)
//
//	���ز�����	ƽ̨���ص�ԭʼ����
//
//	˵����		��ͬ�����豸���صĸ�ʽ��ͬ����Ҫȥ����
//				��ESP8266�ķ��ظ�ʽΪ	"+IPD,x:yyy"	x�������ݳ��ȣ�yyy����������
//==========================================================
unsigned char *ESP8266_GetIPD(unsigned short timeOut)
{

	char *ptrIPD = NULL;
//	char fake[50] = "+IPD,4:";

	
	do
	{
		if(ESP8266_WaitRecive() == REV_OK)								//����������
		{
			ptrIPD = strstr((char *)esp8266_buf, "IPD,");				//������IPD��ͷ
//				ptrIPD = fake;
			if(ptrIPD == NULL)											//���û�ҵ���������IPDͷ���ӳ٣�������Ҫ�ȴ�һ�ᣬ�����ᳬ���趨��ʱ��
			{
//				UsartPrintf(USART_DEBUG, "\"IPD\" not found\r\n");
			}
			else
			{
				ptrIPD = strchr(ptrIPD, ':');							//�ҵ�':'
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
		//delay_ms(5);													//��ʱ�ȴ�
	} while(timeOut--);
	
	return NULL;														//��ʱ��δ�ҵ������ؿ�ָ��

}

//==========================================================
//	�������ƣ�	ESP8266_Init
//
//	�������ܣ�	��ʼ��ESP8266
//
//	��ڲ�����	��
//
//	���ز�����	��
//
//	˵����		
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
	
//	UsartPrintf(USART_DEBUG, "3. AT+CWDHCP\r\n");       //����DHCP(��ȡ����IP),Ĭ�ϱ���Flash
//	while(ESP8266_SendCmd("AT+CWDHCP=1,1\r\n", "OK"))  
//		HAL_Delay(500);

	
//	UsartPrintf(USART_DEBUG, "4. CWJAP\r\n");           //����WIFI
	while(ESP8266_SendCmd(ESP8266_WIFI_INFO, "OK"))
		HAL_Delay(500);
	//	delay_ms(500);
	
//	UsartPrintf(USART_DEBUG, "5. CIPSTART\r\n");        //����TCP����
	while(ESP8266_SendCmd(ESP8266_ONENET_INFO, "CONNECT"))
		HAL_Delay(500);
	//delay_ms(500);
	
//	UsartPrintf(USART_DEBUG, "6. ESP8266 Init OK\r\n"); 

}

//==========================================================
//	�������ƣ�	HAL_UART_RxCpltCallback
//
//	�������ܣ�	����2�շ��ж�
//
//	��ڲ�����	��
//
//	���ز�����	��
//
//	˵����		
//==========================================================
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
////	uint16_t tempt1  /*������ʱ������Ž��ܵ�����*/;
////      if(huart == &huart1)
////    {
////			 tempt1=USART1_RXbuff;
////		 Zigbee_Receive(USART1_RXbuff);	
////        //�����յ������ݷ���
////        //HAL_UART_Transmit(huart,(void *)&USART1_RXbuff, sizeof(USART1_RXbuff),0);
////        //����ʹ�ܴ��ڽ����ж�
////        HAL_UART_Receive_IT(huart,(void *)&USART1_RXbuff, 1);
////    }
//    UNUSED(huart);
//	if(huart == &huart3)//esp8266������ƽ̨����
//		{
//			if(esp8266_cnt >= sizeof(esp8266_buf))  //����ж�
//				{
//					esp8266_cnt = 0;
//					memset(esp8266_buf,0x00,sizeof(esp8266_buf));
//					HAL_UART_Transmit(&huart3, (uint8_t *)"���ջ������", 10,0xFFFF); 	      
//				}
//	 else
//		 {
//			 esp8266_buf[esp8266_cnt++] = aRxBuffer;   //��������ת��	
////		  if(aRxBuffer=='1')  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);	
////        if(aRxBuffer=='0')  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
// 	    }
//	
//	HAL_UART_Receive_IT(&huart3, (uint8_t *)&aRxBuffer, 1);   //�ٿ��������ж�

//		}
//}


//==========================================================
//	�������ƣ�	UsartPrintf
//
//	�������ܣ�	���ڴ�ӡ
//
//	��ڲ�����	��
//
//	���ز�����	��
//
//	˵����		
//========================================================
void UsartPrintf(UART_HandleTypeDef USARTx, char *fmt,...)
{
 
	unsigned char UsartPrintfBuf[296];
	va_list ap;
	unsigned char *pStr = UsartPrintfBuf;
	
	va_start(ap, fmt);
	vsnprintf((char *)UsartPrintfBuf, sizeof(UsartPrintfBuf), fmt, ap);							//��ʽ��
	va_end(ap);
	
	while(*pStr != NULL)
	{
        HAL_UART_Transmit (&USARTx ,(uint8_t *)pStr++,1,HAL_MAX_DELAY );		
	}
 
}

//==========================================================
//	�������ƣ�	Usart_SendString
//
//	�������ܣ�	ʹ�ô��ڷ�������
//
//	��ڲ�����	��
//
//	���ز�����	��
//
//	˵����		
//======================================================
void Usart_SendString(UART_HandleTypeDef USARTx, unsigned char *str, unsigned short len)
{

	unsigned short count = 0;
	
	for(; count < len; count++)
	{
		HAL_UART_Transmit (&USARTx ,(uint8_t *)str++,1,HAL_MAX_DELAY );									//��������
	}

}

