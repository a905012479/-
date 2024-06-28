/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdint.h>

#include "ds1302.h"
#include "OLED_hal.h"
#include "input_hal.h"
#include "servo.h"
#include "solar.h"
#include "gps.h"

#include "IIC.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#include "esp8266.h"
#include "MqttKit.h"
#include "onenet.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PANEL_THRESHOLD 1000
#define SLEEP_TIME 10
#define COMMUNICATION_TIME 15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
bool state_b12, state_b1;

bool steady;
bool fan;

uint8_t dot_value;
uint8_t dot_Y_position;
uint8_t dot_X_position;
uint8_t figure;

double Alt, Azi;
double Lati = 30;
double Longi = 121;
double Final_Azi, Final_Alt;
uint8_t Trace_Mode;                // 0 solar, 1 artifical, 2 sleep

uint16_t sleep_time;

bool Detect_State;
bool Detect_State1;
bool a;
char OLED[17];
uint8_t uart_r_byte[1];
uint16_t uart_r_num;
bool uart_r_over;
bool uart_r_gps_ok;
bool uart_r_$;
bool uart_r_head;
uint8_t uart_r_more_4;
uint8_t OTA_buff[500];
uint8_t start_buff[10];

uint16_t ADC[5];
uint16_t Light_Sensor[5];
uint16_t Light_Intensity[360][4] = {0};
typedef struct {
    int16_t Alt;
    int16_t Azi;
} Offset;

Mgps_msg Mgps;
Offset offset;
Offset Light_Source_Angle;

float pitch,roll,yaw; 		    //娆ф媺瑙?
int angle_Z;
int yaw_offset;

char head1[3];
char temp[50]; //?????????1,?????
char tempAll[100]; //?????????2,??????
int dataLen; //????
unsigned short timeCount = 0;	//发送间隔变量
unsigned char *dataPtr = NULL;
extern uint8_t aRxBuffer;
extern unsigned char esp8266_buf[128];
extern unsigned short esp8266_cnt, esp8266_cntPre;


double Brightest_Val = 4095;
double Sensor_aver = 0;
double Sensor_result = 0;

uint16_t update = 1999;

uint8_t Battery;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Screen_choose_source(void);
void Screen_position_input(double *, double *, uint8_t *, uint8_t *);
void Screen_GPS(void);
void Screen_dot_Y_display(uint8_t);
void Screen_dot_X_display(uint8_t, uint8_t);
void Screen_figure_input(uint8_t, uint8_t, uint8_t, uint8_t *, uint8_t *);

void Screen_running(void);

void GPS_Proc(Mgps_msg Mgps, double *latitude, double *longitude);

uint16_t Find_Light_Source(bool mode);
uint8_t Determine_Light(void);
void Correction(void);
bool Intensity_Detect(void);

void Algorithm_Trace(void);
void Sensor_Trace(void);
void Sleep(void);

float Sensor_Smooth(uint8_t sensor_number);
void Figure_inte(double *, double *, uint16_t *, uint16_t *, int16_t *, int16_t *, uint8_t *, uint8_t *);

void TEST_Sensor_Disp(void);
void TEST_Angle_Disp(void);

int MPU6050_Process(int offset);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
    HAL_UART_Receive_IT(&huart1, uart_r_byte, 1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)Light_Sensor, 5);
	
	Battery = Light_Sensor[4] / 3722.73 * 100.0;
//	while(1)
//	{
//	for(int i = 0; i < 4; i++)
//	{
//		Light_Sensor[i] = ADC[i];
//	}
//}

    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	
	HAL_UART_Receive_IT(&huart3, (uint8_t *)&aRxBuffer, 1); //串口2接收中断初始化
	
	ds1302_gpio_init();
    //    ds1032_init();
	        ds1032_read_realTime();
	HAL_Delay(100);
    OLED_Init();
	
    OLED_ShowString(2, 4, "Initial");
	
    ESP8266_Init();  //8266初始化
	
    while(OneNet_DevLink())  //接入onenet
		continue;
	
	
//		
//	if(++timeCount >= 1)									//上传数据约3s一次
//		{
  
//			UsartPrintf(USART_DEBUG, "OneNet_SendData\r\n");
	        OneNet_SendData( );		
//			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);			
			timeCount = 0;
			ESP8266_Clear( );
//		}
		
        
		  dataPtr = ESP8266_GetIPD(3);//完成需要15个毫秒，三次循环，一次5个毫秒   
		  HAL_Delay(10); 
    
//		  if(dataPtr != NULL)
//          {                         
	      OneNet_RevPro(dataPtr);
//          }
		  HAL_Delay(10); 


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  	MPU_Init();			//MPU6050鍒濆鍖?
	mpu_dmp_init();		//dmp鍒濆鍖?
	
	 while (1)                // screen while
    {
        char oled_buff[17];

        static uint8_t step = 1;
        static uint8_t step_temp = 0;

        static uint8_t Lati_int, Lati_dec;
        static uint8_t Longi_int, Longi_dec;

        static uint8_t dot_mode, dot_temp;

        static uint16_t COUNTER_temp;

        static uint8_t X_temp, Y_temp, figure_temp[2][5], line = 255, column = 255;

        static uint8_t lati[5], longi[5];

         COUNTER_temp = __HAL_TIM_GET_COUNTER(&htim3);
		
		while(mpu_dmp_get_data(&pitch, &roll, &yaw))	//蹇呴』瑕佺敤while绛夊緟锛屾墠鑳借鍙栨垚鍔?
			continue;
		


        if (dot_temp != dot_mode)
        {
            dot_temp = dot_mode;
            if (dot_mode == 0)
            {
                line = 255;
                column = 255;
                OLED_ShowString(2, 1, "              ");
                OLED_ShowString(4, 1, "              ");
                figure_temp[dot_Y_position][dot_X_position] = COUNTER_temp;
                __HAL_TIM_SET_COUNTER(&htim3, Y_temp);
            }
            else if (dot_mode == 1)
            {
                line = dot_Y_position;
                OLED_ShowChar(3, 15, ' ');
                OLED_ShowChar(1, 15, ' ');
                Y_temp = COUNTER_temp;
                __HAL_TIM_SET_COUNTER(&htim3, X_temp);
            }
            else if (dot_mode == 2)
            {
                column = dot_X_position;
                X_temp = COUNTER_temp;
                __HAL_TIM_SET_COUNTER(&htim3, figure_temp[dot_Y_position][dot_X_position]);
            }
        }

        if (dot_mode == 0)
        {
            dot_Y_position = __HAL_TIM_GET_COUNTER(&htim3) / 2 % 2;
        }
        else if (dot_mode == 1)
        {
            dot_X_position = __HAL_TIM_GET_COUNTER(&htim3) / 2 % 5;
        }
        else if (dot_mode == 2)
        {
            figure = __HAL_TIM_GET_COUNTER(&htim3) / 2 % 10;
        }

        if (step_temp != step)
        {
            step_temp = step;
            OLED_Clear();
        }

        switch (step)
        {
            case 1 :
            {
                dot_Y_position = __HAL_TIM_GET_COUNTER(&htim3) / 2 % 2;
                Screen_choose_source();
                Screen_dot_Y_display(dot_Y_position);

                if (Falling_Edge_Detect(&state_b12, GPIOB, GPIO_PIN_12, 10))
                {
                    if (dot_Y_position == 0)                // GPS
                    {
                        step = 20;
						HAL_UART_Receive_IT(&huart1, uart_r_byte, 1);	
                    }
                    if (dot_Y_position == 1)                // manual
                    {
                        step = 30;
                        sprintf(oled_buff, "Lati:  %03u.%02u", Lati_int, Lati_dec);
                        OLED_ShowString(1, 1, oled_buff);
                        sprintf(oled_buff, "Longi: %03u.%02u", Longi_int, Longi_dec);
                        OLED_ShowString(3, 1, oled_buff);
                    }
                    __HAL_TIM_SET_COUNTER(&htim3, 0);
                }
                break;
            }
            case 20 :
            {

                Screen_GPS();
                GPS_Proc(Mgps, &Lati, &Longi);
                if (uart_r_gps_ok)
                {
                    step = 0;
                }
				break;
            }
            case 30 :
            {
                if (Falling_Edge_Detect(&state_b12, GPIOB, GPIO_PIN_12, 10))
                {
                    dot_mode++;
                    if (dot_mode >= 3)
                    {
                        dot_mode = 0;
                    }
                }
                if(Falling_Edge_Detect(&state_b1, GPIOB, GPIO_PIN_1, 10))
                {
                    step = 0;
                }

                if (dot_mode == 0)
                {
                    Screen_dot_Y_display(dot_Y_position);
                }
                else if (dot_mode == 1 && (line == 0 || line == 1))
                {
                    Screen_dot_X_display(dot_X_position, line);
                }
                else if (dot_mode == 2)
                {
                    if (column < 5)
                    {
                        Screen_figure_input(figure, line, 4 - column, lati, longi);
                    }
                }

                Screen_position_input(&Lati, &Longi, lati, longi);
				break;
            }
        }

        if (step == 0)
        {
            OLED_Clear();
            Screen_running();
            break;
        }

        if (Lati > 180)
        {
            Lati = 180.000;
            __HAL_TIM_SET_COUNTER(&htim3, 0);
        }
        if (Longi > 360)
        {
            Longi = 360.000;
            __HAL_TIM_SET_COUNTER(&htim3, 0);
        }
    }

    Correction();
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		Battery = Light_Sensor[4] / 3722.73 * 100.0;

		if(++update == 2000)
		{
			//	if(++timeCount >= 1)									//上传数据约3s一次
//		{
  
//			UsartPrintf(USART_DEBUG, "OneNet_SendData\r\n");
	        OneNet_SendData( );		
//			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);			
			timeCount = 0;
			ESP8266_Clear( );
//		}
		
        
		  dataPtr = ESP8266_GetIPD(3);//完成需要15个毫秒，三次循环，一次5个毫秒   
		  HAL_Delay(10); 
    
//		  if(dataPtr != NULL)
//          {                         
	      OneNet_RevPro(dataPtr);
//          }
		  HAL_Delay(10); 
			
		update = 0;
		}
		


		
		
		if(Falling_Edge_Detect(&state_b12, GPIOB, GPIO_PIN_12, 10))
		{
			steady ^= 1;
			while(mpu_dmp_get_data(&pitch, &roll, &yaw))	//蹇呴』瑕佺敤while绛夊緟锛屾墠鑳借鍙栨垚鍔?
				continue;
			yaw_offset = yaw;
			
		}
		
		if(steady)
		{
			angle_Z = MPU6050_Process(yaw_offset);
		}
		else
		{
			angle_Z = 0;
		}

        Screen_running();
        ds1032_read_realTime();
        Get_Sun(Lati, Longi, &Alt, &Azi);

        Final_Azi = (int)((Azi) - offset.Azi - angle_Z) % 360;
        Final_Alt = ((Alt) - offset.Alt);
		if(Final_Alt < 58)
		{
			Final_Alt = 58;
		}
		else if (Final_Alt > 101)
		{
			Final_Alt = 101;
		}

        if (!steady && Falling_Edge_Detect_Soft(&Detect_State, Intensity_Detect()))
        {
            HAL_Delay(1000);

            if (Intensity_Detect() == 0)
            {
                Trace_Mode = Determine_Light();
            }
        }

        switch (Trace_Mode)
        {
            case 0 :
            {
                Algorithm_Trace();
                break;
            }
            case 1 :
            {
                Sensor_Trace();
                break;
            }
            case 2 :
            {
                Sleep();
                Trace_Mode = 3;
                break;
            }
            default :
            {
                if (sleep_time == SLEEP_TIME)
                {
                    HAL_TIM_Base_Stop(&htim1);
                    sleep_time = 0;
                    Trace_Mode = Determine_Light();
                }
            }
        }
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Screen_choose_source(void)
{
    OLED_ShowString(1, 1, "GPS");
    OLED_ShowString(3, 1, "Manual");
}

void Screen_position_input(double *Lati, double *Longi, uint8_t *lati, uint8_t *longi)
{
    char oled_buff[17];
    static int16_t Lati_int, Longi_int;
    static uint16_t Lati_dec, Longi_dec;

    Figure_inte(Lati, Longi, &Lati_dec, &Longi_dec, &Lati_int, &Longi_int, lati, longi);

    sprintf(oled_buff, "Lati:  %03u.%02u", (uint16_t)*Lati, (uint16_t)(*Lati * 100) % 100);
    OLED_ShowString(1, 1, oled_buff);
    sprintf(oled_buff, "Longi: %03u.%02u", (uint16_t)*Longi, (uint16_t)(*Longi * 100) % 100);
    OLED_ShowString(3, 1, oled_buff);
}

void Figure_inte(double *Lati, double *Longi, uint16_t *Lati_dec, uint16_t *Longi_dec, int16_t *Lati_int, int16_t *Longi_int, uint8_t *lati,
                 uint8_t *longi)
{
    char oled_buff[17];
    static double lati_temp[5], longi_temp[5];
    for (int i = 0; i < 5; i++)
    {
        if (lati_temp[i] != lati[i] || longi_temp[i] != longi[i])
        {
            for (int i = 2; i < 5; i++)
            {
                *Lati_int += lati[i] * pow(10, i - 2);
                *Longi_int += longi[i] * pow(10, i - 2);
            }
            for (int i = 0; i < 2; i++)
            {
                *Lati_dec += lati[i] * pow(10, i);
                *Longi_dec += longi[i] * pow(10, i);
            }
            lati_temp[i] = lati[i];
            longi_temp[i] = longi[i];
            *Lati = *Lati_int + ((double)*Lati_dec * 0.01);
            *Longi = *Longi_int + ((double)*Longi_dec * 0.01);
            *Lati_int = 0;
            *Longi_int = 0;
            *Lati_dec = 0;
            *Longi_dec = 0;
        }
    }
}

void Screen_GPS(void)
{
    OLED_ShowString(2, 4, "Searching");
    OLED_ShowString(3, 4, "Signals...");
}

void Screen_dot_Y_display(uint8_t Y)
{
    if (Y == 0)
    {
        OLED_ShowChar(3, 15, ' ');
        OLED_ShowChar(1, 15, '*');
    }
    else if (Y == 1)
    {
        OLED_ShowChar(1, 15, ' ');
        OLED_ShowChar(3, 15, '*');
    }
}

void Screen_dot_X_display(uint8_t X, uint8_t line)
{
    if (line == 0)
    {
        OLED_ShowString(4, 1, "               ");
        line = 2;
    }
    else if (line == 1)
    {
        OLED_ShowString(2, 1, "               ");
        line = 4;
    }

    switch (X)
    {
        case 0 :
        {
            OLED_ShowString(line, 8, "*     ");
            break;
        }
        case 1 :
        {
            OLED_ShowString(line, 8, " * ");
            break;
        }
        case 2 :
        {
            OLED_ShowString(line, 8, "  *  ");
            break;
        }
        case 3 :
        {
            OLED_ShowString(line, 10, "  * ");
            break;
        }
        case 4 :
        {
            OLED_ShowString(line, 8, "     *");
            break;
        }
    }
}

void Screen_figure_input(uint8_t figure, uint8_t line, uint8_t column, uint8_t *lagi, uint8_t *longi)
{
    switch (line)
    {
        case 0 :
        {
            lagi[column] = figure;
            break;
        }
        case 1 :
        {
            longi[column] = figure;
            break;
        }
    }
}

void Screen_running(void)
{
    char oled_buff[17];
    sprintf(oled_buff, "Lati:  %.2f ", Lati);
    OLED_ShowString(1, 1, oled_buff);
    sprintf(oled_buff, "Longi: %.2f ", Longi);
    OLED_ShowString(2, 1, oled_buff);
	sprintf(oled_buff, "Mode:%u BAT:%d ", Trace_Mode, Battery);
    OLED_ShowString(3, 1, oled_buff);
	
	sprintf(oled_buff, "%.2f  %.2f ", Final_Alt, Final_Azi);
    OLED_ShowString(4, 1, oled_buff);
	
}

int MPU6050_Process(int offset)
{   
    int result = 0;
    for(int i = 0; i < 5; i++)
    {
        while(mpu_dmp_get_data(&pitch, &roll, &yaw))	//蹇呴』瑕佺敤while绛夊緟锛屾墠鑳借鍙栨垚鍔?
		    continue;
        result += (int)((int)yaw - (int)offset);
    }
    return result / 5;
    
}

void GPS_Proc(Mgps_msg Mgps, double *latitude, double *longitude)
{
    if (uart_r_over)
    {
        uart_r_head = 0;
        uart_r_num = 0;
        uart_r_over = 0;
        for (int i = 0; i < 6; i++)
        {
            start_buff[i] = '0';
        }
        for (int i = 0; i < 500; i++)
        {
            OTA_buff[i] = '0';
        }

        HAL_UART_Receive_IT(&huart1, uart_r_byte, 1);
    }
    if (uart_r_gps_ok)
    {
        Andly_GPS(&Mgps, OTA_buff);
    }
    *latitude = atof((const char *)Mgps.latitude) / 100;
    *longitude = atof((const char *)Mgps.longitude) / 100;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (!uart_r_head)                // 锟?????????娴嬪紑锟?????????
        {
            if (uart_r_byte[0] == '$')
            {
                uart_r_$ = 1;
            }
            HAL_UART_Receive_IT(&huart1, uart_r_byte, 1);

            if (uart_r_$)                // 鍒ゆ柇鏄惁锟?????????$GPRMC
            {
                if (uart_r_num < 6)
                {
                    start_buff[uart_r_num] = uart_r_byte[0];
                    uart_r_num++;
                }
                else
                {
                    if (strstr((const char *)start_buff, "$GPRMC") != NULL)
                    {
                        strncpy((char *)OTA_buff, (const char *)start_buff, 6);
                        uart_r_head = 1;
                    }
                    else
                    {
                        uart_r_num = 0;
                    }
                    uart_r_$ = 0;                // 娓呯┖
                    for (int i = 0; i < 6; i++)
                    {
                        start_buff[i] = '0';
                    }
                }
                HAL_UART_Receive_IT(&huart1, uart_r_byte, 1);
            }
        }

        if (uart_r_head && uart_r_num < 500)
        {

            OTA_buff[uart_r_num] = uart_r_byte[0];
            uart_r_num++;
            if (strstr((const char *)OTA_buff, "GPGLL") != NULL)
            {
                uart_r_more_4++;
            }
            if (uart_r_more_4 == 4)
            {
                if (strstr((const char *)OTA_buff, "GPGLL,,,") != NULL)
                {
                    uart_r_over = 1;
                }
                else
                {
                    uart_r_gps_ok = 1;
                }
                uart_r_more_4 = 0;
            }
            else
            {
                HAL_UART_Receive_IT(&huart1, uart_r_byte, 1);                // 閲嶆柊锟?????????鍚覆锟?????????3鎺ユ敹涓柇
            }
        }
    }
	
	
//	uint16_t tempt1  /*定义临时变量存放接受的数据*/;
//      if(huart == &huart1)
//    {
//			 tempt1=USART1_RXbuff;
//		 Zigbee_Receive(USART1_RXbuff);	
//        //将接收到的数据发送
//        //HAL_UART_Transmit(huart,(void *)&USART1_RXbuff, sizeof(USART1_RXbuff),0);
//        //重新使能串口接收中断
//        HAL_UART_Receive_IT(huart,(void *)&USART1_RXbuff, 1);
//    }
    UNUSED(huart);
	if(huart == &huart3)//esp8266接收云平台数据
		{
			if(esp8266_cnt >= sizeof(esp8266_buf))  //溢出判断
				{
					esp8266_cnt = 0;
					memset(esp8266_buf,0x00,sizeof(esp8266_buf));
					HAL_UART_Transmit(&huart3, (uint8_t *)"接收缓存溢出", 10,0xFFFF); 	      
				}
	 else
		 {
			 esp8266_buf[esp8266_cnt++] = aRxBuffer;   //接收数据转存	
//		  if(aRxBuffer=='1')  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);	
//        if(aRxBuffer=='0')  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
 	    }
	
	HAL_UART_Receive_IT(&huart3, (uint8_t *)&aRxBuffer, 1);   //再开启接收中断

		}
}

uint16_t Find_Light_Source(bool mode)
{
    uint16_t angle;
    uint32_t channel;
    uint16_t Brightest_Angle = 0;
	int i = 0;
	
	Brightest_Val = 0;
    Sensor_aver = 0;
	Sensor_result = 0;




    uint32_t sum[4] = {0};
    double Sensor_Group[2];                // 姘村钩鏃讹紝0涓轰笂锟???????1涓轰笅锛涘瀭鐩存椂锟???????0涓哄乏锟???????1涓哄彸
                                           //    double Aver_Ref[4];
                                           //    double Error[4];

    if (mode)
    {
        angle = 360;
        channel = TIM_CHANNEL_1;
    }
    else
    {
        angle = 101;
		i = 58;
        channel = TIM_CHANNEL_2;
    }

    for (; i < angle; i++)
    {
		if(angle != 360)
		{
			Servo_Angle(i, 180, &htim2, channel);

		}
		else
		{
			Servo_Angle(i, angle, &htim2, channel);
		}
        
        HAL_Delay(10);
        Sensor_aver = 0;

        for (int j = 0; j < 4; j++)
        {
            Light_Intensity[i][j] = Sensor_Smooth(j);                // 璁板綍姣忎竴瑙掑害锛屽洓涓紶鎰熷櫒鐨勶拷??     鍔犲钩婊戯紒
            sum[j] += Light_Sensor[j];                               // 杞竴鍦堝悗鐨勪寒搴︼拷?锟斤拷?锟斤紝璁＄畻骞冲潎鍊肩敤
            Sensor_aver += Light_Sensor[j];                          // 姣忎竴瑙掑害锛屽洓涓紶鎰熷櫒鐨勫钩鍧囷拷??
        }

        if (mode)
        {
            Sensor_Group[0] = (double)(Light_Intensity[i][1] + Light_Intensity[i][2]) / 2.0;                // 2锟???????3锟???????
            Sensor_Group[1] = (double)(Light_Intensity[i][0] + Light_Intensity[i][3]) / 2.0;                // 1锟???????4锟???????

            Sensor_result = (Sensor_Group[0] < Sensor_Group[1]) ? Sensor_Group[0] : Sensor_Group[1];
            if (Brightest_Val < Sensor_result)                // 璁板綍涓嬩竴鍦堝唴锟?????????澶т寒搴﹀钩鍧囷拷?锟藉拰瀵瑰簲鐨勮锟?????????
            {
                Brightest_Val = Sensor_result;
                Brightest_Angle = i;
            }
        }
        else
        {
            Sensor_aver /= 4;
            if (Brightest_Val < Sensor_aver)                  // 璁板綍涓嬩竴鍦堝唴锟?????????澶т寒搴﹀钩鍧囷拷?锟藉拰瀵瑰簲鐨勮锟?????????
            {
                Brightest_Val = Sensor_aver;
                Brightest_Angle = i;
            }
        }
    }

                                                              //    for (int i = 0; i < 4; i++)
    //    {
    //        Aver_Ref[i] = (double)sum[i] / angle;
    //        Error[i] = Aver_Ref[i] - Light_Intensity[Brightest_Angle][i];
    //    }


    return Brightest_Angle;
}

bool Intensity_Detect(void)
{
    uint16_t sum;
    for (int i = 0; i < 4; i++)
    {
        sum += Light_Sensor[i];
    }
	
    if (sum / 4.0 < PANEL_THRESHOLD)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

void Correction(void)
{
    ds1032_read_realTime();
    Get_Sun(Lati, Longi, &Alt, &Azi);

    Servo_Angle(58, 180, &htim2, TIM_CHANNEL_2);
    Servo_Angle(0, 360, &htim2, TIM_CHANNEL_1);
    HAL_Delay(2000);

    offset.Azi = (Azi) - Find_Light_Source(1);
    Final_Azi = (Azi) - offset.Azi;
    Servo_Angle(Final_Azi, 360, &htim2, TIM_CHANNEL_1);

    HAL_Delay(2000);
    offset.Alt = (Alt) - Find_Light_Source(0);
    Final_Alt = (Alt) - offset.Alt;
			if(Final_Alt < 58)
		{
			Final_Alt = 58;
		}
		else if (Final_Alt > 101)
		{
			Final_Alt = 101;
		}
    Servo_Angle(Final_Alt, 180, &htim2, TIM_CHANNEL_2);
    HAL_Delay(2000);
}

void Algorithm_Trace(void)
{
    Servo_Angle(Final_Azi, 360, &htim2, TIM_CHANNEL_1);

    Servo_Angle(Final_Alt, 180, &htim2, TIM_CHANNEL_2);
}

void Sensor_Trace(void)
{
	if(steady == 1)
	{
	Servo_Angle(Light_Source_Angle.Azi - angle_Z, 360, &htim2, TIM_CHANNEL_1);
	}
}

void Sleep(void)
{
    Servo_Angle(90, 180, &htim2, TIM_CHANNEL_2);
    Servo_Angle(0, 360, &htim2, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    sleep_time++;
    if (sleep_time < SLEEP_TIME)
    {
        HAL_TIM_Base_Start_IT(&htim1);
    }
}

uint8_t Determine_Light(void)                // 0 sun, 1 artifical, 2 sleep
{
    Servo_Angle(58, 180, &htim2, TIM_CHANNEL_2);
    Servo_Angle(0, 360, &htim2, TIM_CHANNEL_1);
    HAL_Delay(2000);

    Light_Source_Angle.Azi = Find_Light_Source(1);
    Servo_Angle(Light_Source_Angle.Azi, 360, &htim2, TIM_CHANNEL_1);
    HAL_Delay(2000);

    Light_Source_Angle.Alt = Find_Light_Source(0);
		if(Light_Source_Angle.Alt < 58)
		{
			Light_Source_Angle.Alt = 58;
		}
		else if (Light_Source_Angle.Alt > 101)
		{
			Light_Source_Angle.Alt = 101;
		}
    Servo_Angle(Light_Source_Angle.Alt, 180, &htim2, TIM_CHANNEL_2);
    HAL_Delay(1000);

    a = Intensity_Detect();

    if (abs(Light_Source_Angle.Azi - (int)Final_Azi) < 5 && abs(Light_Source_Angle.Alt - (int)Final_Alt) < 5 && a)
    {
        return 0;
    }
    else if (a)
    {
        return 1;
    }
    else
    {
        return 2;
    }
}

float Sensor_Smooth(uint8_t sensor_number)
{
    float ret_val;
    for (int i = 0; i < 20; i++)
    {
        ret_val += Light_Sensor[sensor_number];
    }

    return ret_val / 20;
}

void TEST_Sensor_Disp(void)
{
    sprintf((char *)OLED, "%4d", Light_Sensor[0]);
    OLED_ShowString(1, 1, OLED);
    sprintf((char *)OLED, "%4d", Light_Sensor[1]);
    OLED_ShowString(2, 1, OLED);
    sprintf((char *)OLED, "%4d", Light_Sensor[2]);
    OLED_ShowString(3, 1, OLED);
    sprintf((char *)OLED, "%4d", Light_Sensor[3]);
    OLED_ShowString(4, 1, OLED);
	
}

void TEST_Angle_Disp(void)
{
    sprintf(OLED, "%.2f", Alt);
    OLED_ShowString(1, 1, OLED);
    sprintf(OLED, "%.2f", Azi);
    OLED_ShowString(2, 1, OLED);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
