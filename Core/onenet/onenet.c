#include "main.h"
//网络设备
#include "esp8266.h"

//协议文件
#include "onenet.h"
#include "mqttkit.h"

//C库
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#define PROID		"616145"

#define AUTH_INFO	"thfsb"

#define DEVID		"1207546171"


extern uint8_t temperature; 
extern uint8_t humidity;
extern uint16_t ADC_Sample ,ADC_Volt ;
extern double Lati;
extern double Longi;
extern double Final_Azi, Final_Alt;
extern uint8_t Trace_Mode;
extern uint8_t Battery;

extern unsigned char esp8266_buf[128];


//==========================================================
//	函数名称：	OneNet_DevLink
//
//	函数功能：	与onenet创建连接
//
//	入口参数：	无
//
//	返回参数：	1-成功	0-失败
//
//	说明：		与onenet平台建立连接
//==========================================================
_Bool OneNet_DevLink(void)
{
	
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};					//协议包

	unsigned char *dataPtr;
	
	_Bool status = 1;
	static int time = 0;
	
//	UsartPrintf(USART_DEBUG, "OneNet_DevLink\r\n"
//							"PROID: %s,	AUIF: %s,	DEVID:%s\r\n"
//                        , PROID, AUTH_INFO, DEVID);
	
	if(MQTT_PacketConnect(PROID, AUTH_INFO, DEVID, 256, 0, MQTT_QOS_LEVEL0, NULL, NULL, 0, &mqttPacket) == 0)
	{
		ESP8266_SendData(mqttPacket._data, mqttPacket._len);			//上传平台
		
		dataPtr = ESP8266_GetIPD(1);									//等待平台响应
		
		HAL_Delay(10);
		time++;
		if(time > 15)
		{
			return 0;
		}
//		if(dataPtr != NULL)
//		{
//			if(MQTT_UnPacketRecv(dataPtr) == MQTT_PKT_CONNACK)
//			{

//				switch(MQTT_UnPacketConnectAck(dataPtr))
//				{
//					case 0:
//						status = 0;
//					break;					

//					
//					default:break;
//				}
//			}
//		}
		
		MQTT_DeleteBuffer(&mqttPacket);								//删包
	}
//	else
//		UsartPrintf(USART_DEBUG, "WARN:	MQTT_PacketConnect Failed\r\n");
	
	return status;
	
}

unsigned char OneNet_FillBuf(char *buf)
{
	
	char text[32];

	memset(text, 0, sizeof(text));
	
	strcpy(buf, ",;");
////	Temp = data[2]+(float)data[3]/10;
////    Humidity=data[0]+(float)data[1]/10;
	memset(text, 0, sizeof(text));
	sprintf(text, "Longi,%.2f;",Longi);
	strcat(buf, text);
	
	memset(text, 0, sizeof(text));
	sprintf(text, "Lati,%.2f;",Lati);
	strcat(buf, text);
	
	memset(text, 0, sizeof(text));
	sprintf(text, "360,%.2f;",Final_Azi);
	strcat(buf, text);
	
	memset(text, 0, sizeof(text));
	sprintf(text, "180,%.2f;",Final_Alt);
	strcat(buf, text);
	
	memset(text, 0, sizeof(text));
	sprintf(text, "Battery,%d%%;",Battery);
	strcat(buf, text);
//		memset(text, 0, sizeof(text));
//	
//		strcpy(buf, "{");
//	
    memset(text, 0, sizeof(text));
	sprintf(text, "Mode,%d,",Trace_Mode); 
    strcat(buf, text);
//		
//		memset(text, 0, sizeof(text));
//		sprintf(text, "\"Humidity\":%d,",humidity); 
//    strcat(buf, text);


//		memset(text, 0, sizeof(text));
//		sprintf(text, "\"lux\":%d",ADC_Sample); 
//    strcat(buf, text);


//		strcat(buf, "}");

	
	
	return strlen(buf);

}

//==========================================================
//	函数名称：	OneNet_SendData
//
//	函数功能：	上传数据到平台
//
//	入口参数：	type：发送数据的格式
//
//	返回参数：	无
//
//	说明：		
//==========================================================
void OneNet_SendData(void)
{
	
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};												//协议包
	
	char buf[128];
	
	short body_len = 0, i = 0;
	
//	UsartPrintf(USART_DEBUG, "Tips:	OneNet_SendData-MQTT\r\n");
	
	memset(buf, 0, sizeof(buf));
	
	body_len = OneNet_FillBuf(buf);																	//获取当前需要发送的数据流的总长度
	
	if(body_len)
	{
		if(MQTT_PacketSaveData(DEVID, body_len, NULL, 5, &mqttPacket) == 0)							//封包
		{
			for(; i < body_len; i++)
				mqttPacket._data[mqttPacket._len++] = buf[i];
			
			ESP8266_SendData(mqttPacket._data, mqttPacket._len);									//上传数据到平台
//			UsartPrintf(USART_DEBUG, "Send %d Bytes\r\n", mqttPacket._len);
//			
			MQTT_DeleteBuffer(&mqttPacket);															//删包
		}
		else
		{
		}
//			UsartPrintf(USART_DEBUG, "WARN:	EDP_NewBuffer Failed\r\n");
	}
	
}

//==========================================================
//	函数名称：	OneNet_RevPro
//
//	函数功能：	平台返回数据检测
//
//	入口参数：	dataPtr：平台返回的数据
//
//	返回参数：	无
//
//	说明：		
//==========================================================
void OneNet_RevPro(unsigned char *cmd)
{
	
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};								//协议包
	
	char *req_payload = NULL;
	char *cmdid_topic = NULL;
	
	unsigned short req_len = 0;
	
	unsigned char type = 0;
	
	short result = 0;

	char *dataPtr = NULL;
	char numBuf[10];
	int num = 0;
	
	type = MQTT_UnPacketRecv(cmd);
	switch(type)
	{
		case MQTT_PKT_CMD:															//命令下发
			
			result = MQTT_UnPacketCmd(cmd, &cmdid_topic, &req_payload, &req_len);	//解出topic和消息体
			if(result == 0)
			{
//				UsartPrintf(USART_DEBUG, "cmdid: %s, req: %s, req_len: %d\r\n", cmdid_topic, req_payload, req_len);
				
				if(MQTT_PacketCmdResp(cmdid_topic, req_payload, &mqttPacket) == 0)	//命令回复组包
				{
//					UsartPrintf(USART_DEBUG, "Tips:	Send CmdResp\r\n");
					
					ESP8266_SendData(mqttPacket._data, mqttPacket._len);			//回复命令
					MQTT_DeleteBuffer(&mqttPacket);									//删包
				}
			}
		
		break;
			
		case MQTT_PKT_PUBACK:														//发送Publish消息，平台回复的Ack
		
			if(MQTT_UnPacketPublishAck(cmd) == 0)
//				UsartPrintf(USART_DEBUG, "Tips:	MQTT Publish Send OK\r\n");
			
		break;
		
		default:
			result = -1;
		break;
	}
	
	ESP8266_Clear();									//清空缓存
	
	if(result == -1)
		return;
	
	dataPtr = strchr(req_payload, '}');					//搜索'}'

	if(dataPtr != NULL && result != -1)					//如果找到了
	{
		dataPtr++;
		
		while(*dataPtr >= '0' && *dataPtr <= '9')		//判断是否是下发的命令控制数据
		{
			numBuf[num++] = *dataPtr++;
		}
		numBuf[num] = 0;
		
		num = atoi((const char *)numBuf);				//转为数值形式
	}

	if(type == MQTT_PKT_CMD || type == MQTT_PKT_PUBLISH)
	{
		MQTT_FreeBuffer(cmdid_topic);
		MQTT_FreeBuffer(req_payload);
	}

}


