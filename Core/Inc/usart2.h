
#ifndef __USART3_H
#define __USART3_H

#include "stdio.h"      
#include "stdarg.h"		 
#include "string.h"    

#define USART3_TXBUFF_SIZE   1024   		   //定义串口2 发送缓冲区大小 1024字节
#define USART3_RXBUFF_SIZE   1024              //定义串口2 接收缓冲区大小 1024字节

extern char Usart3_RxCompleted ;               //外部声明，其他文件可以调用该变量
extern unsigned int Usart3_RxCounter;          //外部声明，其他文件可以调用该变量
extern char Usart3_RxBuff[USART3_RXBUFF_SIZE]; //外部声明，其他文件可以调用该变量

void USART3_Init(unsigned int);       
void u2_printf(char*,...) ;          
void u2_TxData(unsigned char *data);

#endif


