
/*-------------------------------------------------*/
/*                                                 */
/*          	 WIFI��ESP8266��Դ�ļ�             */
/*                                                 */
/*-------------------------------------------------*/

// Ӳ�����ӣ�
// PA2 RX
// PA3 TX
// PA4 ��λ

#include "stm32f1xx_hal.h"  //������Ҫ��ͷ�ļ�
#include "wifi.h"	    //������Ҫ��ͷ�ļ�
#include "delay.h"	    //������Ҫ��ͷ�ļ�
#include "usart2.h"	    //������Ҫ��ͷ�ļ�

char wifi_mode = 0;     //����ģʽ 0��SSID������д�ڳ�����   1��Smartconfig��ʽ��APP����
	
/*-------------------------------------------------*/
/*����������ʼ��WiFi�ĸ�λIO                       */
/*��  ������                                       */
/*����ֵ����                                       */
/*-------------------------------------------------*/
void WiFi_ResetIO_Init(void)
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	GPIO_InitTypeDef GPIO_InitStructure;
 	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
	GPIO_InitStructure.Pin = GPIO_PIN_3;
 	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	RESET_IO(1);

}
/*-------------------------------------------------*/
/*��������WiFi��������ָ��                          */
/*��  ����cmd��ָ��                                */
/*��  ����timeout����ʱʱ�䣨100ms�ı�����          */
/*����ֵ��0����ȷ   ����������                      */
/*-------------------------------------------------*/
char WiFi_SendCmd(char *cmd, int timeout)
{
	WiFi_RxCounter = 0;                           			//WiFi������������������                        
	memset(WiFi_RX_BUF, 0, WiFi_RXBUFF_SIZE);     			//���WiFi���ջ����� 
	WiFi_printf("%s\r\n", cmd);                  			//����ָ��
	while(timeout--)										//�ȴ���ʱʱ�䵽0
	{                           			
		HAL_Delay(100);                             		    //��ʱ100ms
		if(strstr(WiFi_RX_BUF, "OK"))              			//������յ�OK��ʾָ��ɹ�
			break;       									//��������whileѭ��
		//u1_printf("%d ", timeout);                 			//����������ڵĳ�ʱʱ��
	}			
	//u1_printf("\r\n");                          			//���������Ϣ
	if(timeout <= 0)return 1;                    				//���timeout<=0��˵����ʱʱ�䵽�ˣ�Ҳû���յ�OK������1
	else return 0;		         							//��֮����ʾ��ȷ��˵���յ�OK��ͨ��break��������while
}
/*-------------------------------------------------*/
/*��������WiFi��λ                                 */
/*��  ����timeout����ʱʱ�䣨100ms�ı�����          */
/*����ֵ��0����ȷ   ����������                      */
/*-------------------------------------------------*/
char WiFi_Reset(int timeout)
{
	RESET_IO(0);                                    	  //��λIO���͵�ƽ
	HAL_Delay(500);                                  		  //��ʱ500ms
	RESET_IO(1);                                   		  //��λIO���ߵ�ƽ	
	while(timeout--)									  //�ȴ���ʱʱ�䵽0 
	{                              		  
		HAL_Delay(100);                                 	  //��ʱ100ms
		if(strstr(WiFi_RX_BUF, "ready"))               	  //������յ�ready��ʾ��λ�ɹ�
			break;       						   		  //��������whileѭ��
		//u1_printf("%d ", timeout);                     	  //����������ڵĳ�ʱʱ��
	}
	//u1_printf("\r\n");                              	  //���������Ϣ
	if(timeout <= 0)return 1;                        		  //���timeout<=0��˵����ʱʱ�䵽�ˣ�Ҳû���յ�ready������1
	else return 0;		         				   		  //��֮����ʾ��ȷ��˵���յ�ready��ͨ��break��������while
}
/*-------------------------------------------------*/
/*��������WiFi����·����ָ��                       */
/*��  ����timeout����ʱʱ�䣨1s�ı�����            */
/*����ֵ��0����ȷ   ����������                     */
/*-------------------------------------------------*/
char WiFi_JoinAP(int timeout)
{		
	WiFi_RxCounter = 0;                                    //WiFi������������������                        
	memset(WiFi_RX_BUF, 0, WiFi_RXBUFF_SIZE);              //���WiFi���ջ����� 
	WiFi_printf("AT+CWJAP=\"%s\",\"%s\"\r\n", SSID, PASS); //����ָ��	
	while(timeout--)									   //�ȴ���ʱʱ�䵽0
	{                                   
		HAL_Delay(1000);                             		   //��ʱ1s
		if(strstr(WiFi_RX_BUF, "OK"))   //������յ�WIFI GOT IP��ʾ�ɹ�
			break;       						           //��������whileѭ��
		//u1_printf("%d ", timeout);                         //����������ڵĳ�ʱʱ��
	}
	//u1_printf("\r\n%s\r\n", WiFi_RX_BUF);
	//u1_printf("\r\n");                             	       //���������Ϣ
	if(timeout <= 0)return 1;                              //���timeout<=0��˵����ʱʱ�䵽�ˣ�Ҳû���յ�WIFI GOT IP������1
	return 0;                                              //��ȷ������0
}
/*-------------------------------------------------*/
/*������������TCP��������������͸��ģʽ            */
/*��  ����timeout�� ��ʱʱ�䣨100ms�ı�����        */
/*����ֵ��0����ȷ  ����������                      */
/*-------------------------------------------------*/
char WiFi_Connect_Server(int timeout)
{	
	WiFi_RxCounter=0;                              	//WiFi������������������                        
	memset(WiFi_RX_BUF,0,WiFi_RXBUFF_SIZE);         //���WiFi���ջ�����   
	WiFi_printf("AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", ServerIP, ServerPort);//�������ӷ�����ָ��
	while(timeout--)								//�ȴ���ʱ���
	{                           
		HAL_Delay(100);                             	//��ʱ100ms	
		if(strstr(WiFi_RX_BUF, "CONNECT"))          //������ܵ�CONNECT��ʾ���ӳɹ�
			break;                                  //����whileѭ��
		if(strstr(WiFi_RX_BUF, "CLOSED"))           //������ܵ�CLOSED��ʾ������δ����
			return 1;                               //������δ��������1
		if(strstr(WiFi_RX_BUF, "ALREADY CONNECTED"))//������ܵ�ALREADY CONNECTED�Ѿ���������
			return 2;                               //�Ѿ��������ӷ���2
		//u1_printf("%d ", timeout);                   //����������ڵĳ�ʱʱ��  
	}
	//u1_printf("\r\n");                              //���������Ϣ
	if(timeout <= 0)return 3;                       //��ʱ���󣬷���3
	else                                            //���ӳɹ���׼������͸��
	{
		//u1_printf("���ӷ������ɹ�׼������͸��"); //������ʾ��Ϣ
		//u3_printf("���ӷ������ɹ�");				//������ʾ
		//u3_printf("׼������͸��");				//������ʾ
		WiFi_RxCounter = 0;                          //WiFi������������������                        
		memset(WiFi_RX_BUF, 0, WiFi_RXBUFF_SIZE);    //���WiFi���ջ�����     
		WiFi_printf("AT+CIPSEND\r\n");               //���ͽ���͸��ָ��
		while(timeout--)							 //�ȴ���ʱ���
		{                            
			HAL_Delay(100);                            //��ʱ100ms	
			if(strstr(WiFi_RX_BUF, "\r\nOK\r\n\r\n>"))//���������ʾ����͸���ɹ�
				break;                          	 //����whileѭ��
			//u1_printf("%d ", timeout);                //����������ڵĳ�ʱʱ��  
		}
		if(timeout <= 0)return 4;                      //͸����ʱ���󣬷���4	
	}
	return 0;	                                     //�ɹ�����0	
}
/*-------------------------------------------------*/
/*��������WiFi_Smartconfig                         */
/*��  ����timeout����ʱʱ�䣨1s�ı�����            */
/*����ֵ��0����ȷ   ����������                     */
/*-------------------------------------------------*/
char WiFi_Smartconfig(int timeout)
{
	
	WiFi_RxCounter=0;                           		//WiFi������������������                        
	memset(WiFi_RX_BUF,0,WiFi_RXBUFF_SIZE);     		//���WiFi���ջ�����     
	while(timeout--)									//�ȴ���ʱʱ�䵽0
	{                           		
		HAL_Delay(1000);                         				//��ʱ1s
		if(strstr(WiFi_RX_BUF, "connected"))    	 		  //������ڽ��ܵ�connected��ʾ�ɹ�
			break;                                  		//����whileѭ��  
		//u1_printf("%d ", timeout);                 		//����������ڵĳ�ʱʱ��  
	}	
	//u1_printf("\r\n");                          		//���������Ϣ
	if(timeout <= 0)return 1;                     		//��ʱ���󣬷���1
	return 0;                                   		//��ȷ����0
}
/*-------------------------------------------------*/
/*���������ȴ�����·����                           */
/*��  ����timeout����ʱʱ�䣨1s�ı�����            */
/*����ֵ��0����ȷ   ����������                     */
/*-------------------------------------------------*/
char WiFi_WaitAP(int timeout)
{		
	while(timeout--){                               //�ȴ���ʱʱ�䵽0
		HAL_Delay(1000);                             		//��ʱ1s
		if(strstr(WiFi_RX_BUF, "WIFI GOT IP"))         //������յ�WIFI GOT IP��ʾ�ɹ�
			break;       						   								  //��������whileѭ��
		//u1_printf("%d ", timeout);                     //����������ڵĳ�ʱʱ��
	}
	//u1_printf("\r\n");                              //���������Ϣ
	if(timeout <= 0)return 1;                         //���timeout<=0��˵����ʱʱ�䵽�ˣ�Ҳû���յ�WIFI GOT IP������1
	return 0;                                       //��ȷ������0
}
/*-------------------------------------------------*/
/*��������WiFi���ӷ�����                           */
/*��  ������                                       */
/*����ֵ��0����ȷ   ����������                     */
/*-------------------------------------------------*/
//char WiFi_Connect_IoTServer(void)
//{	
//	//u1_printf("׼����λģ��");                   //������ʾ����
//	if(WiFi_Reset(50))								//��λ��100ms��ʱ��λ���ܼ�5s��ʱʱ��
//	{                             
//		//u1_printf("��λʧ��׼������");	      //���ط�0ֵ������if��������ʾ����
//		return 1;                                   //����1
//	}//else u1_printf("��λ�ɹ�");                 //������ʾ����
//	
//	//u1_printf("׼������STAģʽ");                //������ʾ����
//	if(WiFi_SendCmd("AT+CWMODE=1",50))//����STAģʽ��100ms��ʱ��λ���ܼ�5s��ʱʱ��
//	{             
//		u1_printf("����STAģʽʧ��׼������");   //���ط�0ֵ������if��������ʾ����
//		return 2;                                   //����2
//	}else u1_printf("����STAģʽ�ɹ�");          //������ʾ����
//	
//	if(wifi_mode==0) //�������ģʽ=0��SSID������д�ڳ����� 
//	{                              
//		u1_printf("׼��ȡ���Զ�����");            //������ʾ����
//		if(WiFi_SendCmd("AT+CWAUTOCONN=0",50))		 //ȡ���Զ����ӣ�100ms��ʱ��λ���ܼ�5s��ʱʱ��
//		{       
//			u1_printf("ȡ���Զ�����ʧ�ܣ�׼������"); //���ط�0ֵ������if��������ʾ����
//			return 3;                                  //����3
//		}else u1_printf("ȡ���Զ����ӳɹ�");        //������ʾ����
//				
//		u1_printf("׼������·����");                //������ʾ����	
//		if(WiFi_JoinAP(30))//����·����,1s��ʱ��λ���ܼ�30s��ʱʱ��
//		{                          
//			u1_printf("����·����ʧ��׼������");  //���ط�0ֵ������if��������ʾ����
//			return 4;                                 //����4	
//		}else u1_printf("����·�����ɹ�");         //������ʾ����			
//	}
//	
//	u1_printf("׼������͸��");                    //������ʾ����
//	if(WiFi_SendCmd("AT+CIPMODE=1",50)) 			 //����͸����100ms��ʱ��λ���ܼ�5s��ʱʱ��
//	{           
//		u1_printf("����͸��ʧ��׼������");       //���ط�0ֵ������if��������ʾ����
//		return 8;                                    //����8
//	}else u1_printf("����͸���ɹ�");              //������ʾ����
//	
//	u1_printf("׼���رն�·����");                //������ʾ����
//	if(WiFi_SendCmd("AT+CIPMUX=0",50)) 				 //�رն�·���ӣ�100ms��ʱ��λ���ܼ�5s��ʱʱ��
//	{            
//		u1_printf("�رն�·����ʧ��׼������");   //���ط�0ֵ������if��������ʾ����
//		return 9;                                    //����9
//	}else u1_printf("�رն�·���ӳɹ�");          //������ʾ����
//	 
//	u1_printf("׼�����ӷ�����");                  //������ʾ����
//	if(WiFi_Connect_Server(100))      				 //���ӷ�������100ms��ʱ��λ���ܼ�10s��ʱʱ��
//	{            
//		u1_printf("���ӷ�����ʧ��׼������");     //���ط�0ֵ������if��������ʾ����
//		return 10;                                   //����10
//	}else{
//		u1_printf("���ӷ������ɹ�");            //������ʾ����	
//		//u3_printf("���ӷ������ɹ�");
//	}
//	return 0;                                        //��ȷ����0
//}
	

