#ifndef __gps_H
#define __gps_H
 
 
#include "main.h"
#include "string.h"
#include "stdio.h"
typedef struct  
{                                            
    uint8_t latitude[10];                //纬度 
    uint8_t longitude[11];            //经度     
    uint8_t Location[1];            //定位有效A 定位无效V 
    
    uint8_t lat[20];                //纬度 
    uint8_t lon[20];
 
    uint8_t latt;    //经度     
}Mgps_msg; 
extern  Mgps_msg Mgps;
 
 
 
 
void Andly_GPS(Mgps_msg *Mgps,uint8_t *buf);    
uint8_t NMEA_Comma_Pos(uint8_t *buf,uint8_t cx);
void Gps_Msg_Show(void);//显示GPS定位信息 
int NMEA_Str2num(uint8_t *buf,uint8_t*dx);
 
 
#endif 