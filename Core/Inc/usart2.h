
#ifndef __USART3_H
#define __USART3_H

#include "stdio.h"      
#include "stdarg.h"		 
#include "string.h"    

#define USART3_TXBUFF_SIZE   1024   		   //���崮��2 ���ͻ�������С 1024�ֽ�
#define USART3_RXBUFF_SIZE   1024              //���崮��2 ���ջ�������С 1024�ֽ�

extern char Usart3_RxCompleted ;               //�ⲿ�����������ļ����Ե��øñ���
extern unsigned int Usart3_RxCounter;          //�ⲿ�����������ļ����Ե��øñ���
extern char Usart3_RxBuff[USART3_RXBUFF_SIZE]; //�ⲿ�����������ļ����Ե��øñ���

void USART3_Init(unsigned int);       
void u2_printf(char*,...) ;          
void u2_TxData(unsigned char *data);

#endif


