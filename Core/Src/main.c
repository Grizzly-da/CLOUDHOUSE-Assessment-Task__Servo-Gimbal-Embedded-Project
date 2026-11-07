/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include"string.h"
#include"stdlib.h"
#include"stdio.h"
#include"math.h"
#include"mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define  pitch 1380
#define  yaw 1450
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile uint8_t character; //单字符  
volatile uint8_t recvd[34]; //字符串缓冲
volatile uint8_t  i= 0;//字符串数组角标
volatile uint8_t finish = 0;  //字符串接收完成标志
volatile uint8_t cnt = 1; //模式，会转化为n
uint16_t pitch_angle=90,yaw_angle=90;//舵机初始角
float   gyro_bias[3]  = {0, 0, 0}; //陀螺仪零偏
float pitch0=0,roll0=0;//acc偏移
int MPU_OK=-1;//默认让led1常亮，CAL后0则闪烁，1则常亮
float  Pitch=0.0f,Roll=0.0f;//MPU俯仰角，翻滚角
char Pitch_Roll[48], pitch_yaw[48];//OLED缓冲
uint32_t last_mputick=0,last_oled_tick,lastkeytick=0,now=0;//mpuUpdate mpu_oled 按钮消抖的时间戳
volatile uint16_t initial_adc[4] = {0};   //初始值
uint16_t ADC[4] = {0};   //滤波后的值
uint16_t adc_buf[4][8] = {0};//缓冲
float ADC_pitch = 0.0f;   // ADC相关的舵机角度
float ADC_yaw   = 0.0f;
volatile uint8_t adc_ready = 0;
//追光参数


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
    
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE BEGIN 0 */

float clamp(float a)//钳位函数
{
if(a<0.0f) return 0.0f;
else if(a>180.0) return 180.0f;
else return a;	
}

void MPU_UpdateAttitude(void)
{
   short acc[3],gyro[3];//六轴数据
	 uint32_t now =HAL_GetTick();
	
	 if((now-last_mputick)<20) return;
	 //else 开始更新读数
	 last_mputick=now;
   MPU6050ReadAcc(acc);
   MPU6050ReadGyro(gyro);
	
	  float gx = (gyro[0] - gyro_bias[0]) / 16.4f; 
    float gy = (gyro[1] - gyro_bias[1]) / 16.4f;
	
	  float ax = acc[0] / 16384.0f;
    float ay = acc[1] / 16384.0f;
    float az = acc[2] / 16384.0f;
	  float acc_pitch = atan2f( -ax, sqrtf(ay*ay + az*az)) * 57.3f;
    float acc_roll  = atan2f( ay, sqrtf(ax*ax + az*az)) * 57.3f;
    acc_pitch -= pitch0;
    acc_roll  -= roll0;
	//互补滤波，陀螺仪积分修正，加速度修正
	  const float dt = 0.02f;
    Pitch = 0.98f * (Pitch + gx * dt) +0.02f * acc_pitch;
    Roll  = 0.98f * (Roll  + gy * dt) + 0.02f * acc_roll;
		
		if(cnt==2)
		{
		//把姿态角映射给舵机,判断钳位
      float p=clamp(-1*Pitch),y=clamp(-1*Roll);
		//尽量纠正舵机偏差
      uint32_t PitchCompare = pitch -(uint32_t)((p-90)* 11.0f);
      uint32_t RollCompare  = (uint32_t)(yaw+ (y-90)* 10.5f);
     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,PitchCompare);
     __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,RollCompare);
			
		}
		if(now-last_oled_tick>=50)
		{
		  last_oled_tick=now;
			int p = (int)Pitch,r = (int)Roll;
      if (p > 999) p = 999;
      if (p < -999) p = -999;
      if (r > 999) r = 999;
      if (r < -999) r = -999;
		sprintf(Pitch_Roll, "MPU:PIT %d ROLL %d       ",-1*p,-1*r);
    OLED_ShowString(0, 48, Pitch_Roll, 6);
			 if (cnt == 2)
        {
            sprintf(pitch_yaw, "PIT %d YAW %d   ", (int)clamp(-1*Pitch),(int)clamp(-1*Roll));
            OLED_ShowString(0, 36, pitch_yaw, 6);
        }
    OLED_Update();
		}

}

char Transform(char c)    //abc->ABC
{
    if (c >= 'a' && c <= 'z') c -= 32;
    return c;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc == &hadc1)
    {
        adc_ready = 1;  
    }
}

void  dealCMD(void)
{
	 
    char CMD[40];
	 __disable_irq();
    uint8_t k,length=strlen((const char *)recvd);
	uint32_t Compare=1500;
		for (k = 0; k <length; k++)
        CMD[k] = Transform((char)recvd[k]);
        CMD[k] = '\0';
     __enable_irq();
    if (strcmp(CMD, "AT") == 0)
    {
        OLED_ShowString(0, 24, "OK                  ", 6);
        OLED_Update();
    }
    else if (strcmp(CMD, "STATUS") == 0)
    {
        uint8_t n;
        if (cnt == 0)      n = 3;
        else if (cnt == 1) n = 1;
        else               n = 2;

        char buf[24];
        buf[0]='O'; buf[1]='K'; buf[2]=' '; buf[3]='M';buf[4]='O'; buf[5]='D'; buf[6]='E'; buf[7]='=';
        buf[8]='0' + n; buf[9]=' ';buf[10]=' ';buf[11]=' ';buf[12]=' ';buf[13]=' ';buf[14]=' ';buf[15]='\0';
        OLED_ShowString(0, 24, buf, 6);
        OLED_Update();
    }
		else if(strncmp(CMD,"PITCH ",6)==0||strncmp(CMD,"YAW ",4)==0)
    {
			if(cnt!=1){OLED_ShowString(0,24,"ERR MODE                ",6);OLED_Update();return;}
			int is_pitch=0,Angle=0; 
			if(strncmp(CMD,"YAW ",4)==0)
			{
			is_pitch=0;
			Angle=atoi(&CMD[4]);
			}
			else
			{
			is_pitch=1;
			Angle=atoi(&CMD[6]);
			}
			if(Angle<0||Angle>180)
			{
			OLED_ShowString(0,24,"ERR OverStep   ",6);
			OLED_Update();
				
				return;
			}
		
    if(is_pitch)
		{
			pitch_angle=Angle;
			Compare=pitch+(Angle-90)*11;
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,Compare);
			OLED_ShowString(0,24,"OK                ",6);
			OLED_Update();
		}			
		 else	
		{
      yaw_angle=Angle;
			Compare=(uint32_t)(yaw+(Angle-90)*10.5f);
	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,Compare);
			OLED_ShowString(0,24,"OK                ",6);
			OLED_Update();
		}			
		sprintf(pitch_yaw,"PIT %d YAW %d                   ",pitch_angle,yaw_angle);
		OLED_ShowString(0,36,pitch_yaw,6);
		OLED_Update();
		} 
	else if(strcmp(CMD,"CAL")==0)
    {
    double  gyro_sum[3] = {0, 0, 0};
    double  pitch_sum = 0.0, roll_sum = 0.0;
    short   gyro_min[3], gyro_max[3],acc[3],gyro[3];
    int  m=0;
    if (MPU6050ReadID() == 0)
	   {
	   OLED_ShowString(0,24,"ERR NotFound",6);//attention!!!0x70,Edit this in mpu6050HeadFile
	   OLED_Update();
		 MPU_OK=0;
			
		 return;
	   }
    else
  {  
    //初始化
    MPU6050ReadAcc(acc);
    MPU6050ReadGyro(gyro);
    for (m = 0; m < 3; m++) {
        gyro_min[m] = gyro[m];
        gyro_max[m] = gyro[m];
    }

   //采样200次
    for (m = 0; m< 200; m++)
    {   MPU6050ReadAcc(acc);
        MPU6050ReadGyro(gyro);
        gyro_sum[0] += gyro[0];
        gyro_sum[1] += gyro[1];
        gyro_sum[2] += gyro[2];
        //陀螺轴min/max
        if (gyro[0] < gyro_min[0]) gyro_min[0] = gyro[0];
        if (gyro[0] > gyro_max[0]) gyro_max[0] = gyro[0];
        if (gyro[1] < gyro_min[1]) gyro_min[1] = gyro[1];
        if (gyro[1] > gyro_max[1]) gyro_max[1] = gyro[1];
        if (gyro[2] < gyro_min[2]) gyro_min[2] = gyro[2];
        if (gyro[2] > gyro_max[2]) gyro_max[2] = gyro[2];
        pitch_sum += atan2f(-(float)acc[0],
        sqrtf((float)acc[1]*(float)acc[1] +(float)acc[2]*(float)acc[2])) * 57.3f;
        roll_sum  += atan2f( (float)acc[1],
        sqrtf((float)acc[0]*(float)acc[0] +(float)acc[2]*(float)acc[2])) * 57.3f;
      }
	
	   for (m = 0; m < 3; m++)
    {
        if ((gyro_max[m] - gyro_min[m]) > 300)
		  {
			OLED_ShowString(0,24,"ERR Shake       ",6);
			OLED_Update();
			MPU_OK=0;
				
			return;
      }    
	  }
        short a[3];
        MPU6050ReadAcc(a);
	      float mod = sqrtf(a[2]*a[2]);//Z轴模长
       if (fabsf(fabsf((float)a[2]) - 16384.0f) > 2000.0f)
		 {
	   OLED_ShowString(0,24,"ERR Uneven      ",6);
	   OLED_Update();
		 MPU_OK=0;
			
		 return;
		 }   
		      
      else
	  {
	   OLED_ShowString(0,24,"OK CAL           ",6);
	   OLED_Update();
		 MPU_OK=1;
		gyro_bias[0] = gyro_sum[0] / 200;
    gyro_bias[1] = gyro_sum[1] / 200;
    gyro_bias[2] = gyro_sum[2] / 200;

    pitch0 = pitch_sum / 200;
    roll0  = roll_sum  / 200; 
		
		}
 }
	}
	else
    {
        OLED_ShowString(0, 24, "ERR InvalidCMD      ", 6);
        OLED_Update();
    }
		
}

/*uint16_t oled_check(void)
{
	//SCL HIGH->work,SCL LOW->prepare for data
    //START
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    for(volatile int i=0;i<20;i++) __NOP();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
    for(volatile int i=0;i<20;i++) __NOP();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    for(volatile int i=0;i<20;i++) __NOP();

    // search for locatin 0x78
    uint8_t ack = 0, d = 0x78;
    for(uint8_t i=0;i<8;i++){
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, (d&0x80)?GPIO_PIN_SET:GPIO_PIN_RESET);
        d <<= 1;
        for(volatile int j=0;j<20;j++) __NOP();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
        for(volatile int j=0;j<20;j++) __NOP();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
        for(volatile int j=0;j<20;j++) __NOP();
    }
    //answer ACK
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    for(volatile int j=0;j<20;j++) __NOP();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    for(volatile int j=0;j<20;j++) __NOP();
    ack = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

    //STOP
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
    for(volatile int j=0;j<20;j++) __NOP();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    for(volatile int j=0;j<20;j++) __NOP();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

    if(ack == 0)
	   {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
		return 1;
       }
		else return 0;
		
}*/


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart== &huart1)
    {
        uint8_t c = character;

        if (c == '\r' || c == '\n')
        {
            if (i> 0)             
            {
                recvd[i] = '\0';
                finish = 1;
                i = 0;
            }
            
        }
        else if (i < 32)   
        {
            recvd[i++] = c;
        }
       
    else
   {
    i = 0;
    recvd[0] = '\0';
    finish = 2;
   }
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&character, 1);
    }
}

void dealADC(void)//滤波
{
    static uint32_t sum[4] = {0};
    static uint8_t  ijk = 0;
		
		if (!adc_ready) return;
    adc_ready = 0;
		
		

   
    for (int cycle = 0; cycle < 4; cycle++) 
		{
        sum[cycle] -= adc_buf[cycle][ijk];
        adc_buf[cycle][ijk] = initial_adc[cycle];
        sum[cycle] += initial_adc[cycle];
        ADC[cycle] = (uint16_t)(sum[cycle] / 8);
		}
    ijk = (ijk + 1) % 8;
}


void track_light(void)
{
    float up    = (float)ADC[1];
    float down  = (float)ADC[3];
    float left  = (float)ADC[0];
    float right = (float)ADC[2];

    float Y_bias = up - down;      
    float X_bias = right - left;   

    if (fabsf(Y_bias) < 150.0f) Y_bias = 0.0f;
    if (fabsf(X_bias) < 150.0f) X_bias = 0.0f;
    if (Y_bias == 0.0f && X_bias == 0.0f) return;
    

    ADC_pitch = 0.01f * Y_bias;
    ADC_yaw   = 0.01f * X_bias;
	 
	  static float Y_f = 0.0f, X_f = 0.0f;
    Y_f = 0.7f * Y_f + 0.3f * Y_bias;
    X_f = 0.7f * X_f + 0.3f * X_bias;
    
	  float pitch_delta = 0.012f * Y_f; 
    float yaw_delta   = 0.012f * X_f;
		
		if (pitch_delta >  80.0f) pitch_delta =  80.0f;
    if (pitch_delta < -80.0f) pitch_delta = -80.0f;
    if (yaw_delta   >  80.0f) yaw_delta   =  80.0f;
    if (yaw_delta   < -80.0f) yaw_delta   = -80.0f;
    
		static uint32_t pitch_cmp = pitch, yaw_cmp = yaw;

    int32_t p = (int32_t)pitch_cmp + (int32_t)pitch_delta;
    int32_t y = (int32_t)yaw_cmp   + (int32_t)yaw_delta;

   
    if (p < 800) p = 800;  if (p > 1960) p = 1960;  
    if (y < 750) y = 750;  if (y > 2100) y = 2100;   

    pitch_cmp = (uint32_t)p;
    yaw_cmp   = (uint32_t)y;
  
  
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)pitch_cmp);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint16_t)yaw_cmp);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
   
   
	      uint8_t pre=1, cur=1,press=0;
				uint32_t t0=0,t1=0;
				uint32_t lasttime=0,mpulasttime=0;


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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	
	OLED_Init();
	MPU6050_Init();
	OLED_ShowString(0, 0, "CLOUDHOUSE", 6);
	OLED_ShowString(0, 12, "MODE 1", 6);
	OLED_ShowString(0,36,"PIT 90 YAW 90    ",6);
	OLED_Update();
	
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&character, 1);
  
  for (int j = 0; j < 3; j++)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
        
        HAL_Delay(200);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
			
        HAL_Delay(200);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
        
	      HAL_Delay(200);
	      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
		
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//PA8 Control PITCH,1380
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);//PB3 Control YAW 600~1450
	
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)initial_adc,4);//启动ADC1



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
		
		
		/* USER CODE BEGIN WHILE */
        
		    

while (1)
{
     dealADC();
	   if (cnt == 0) 
		{
			static uint32_t last_track = 0;
      if (HAL_GetTick() - last_track >= 20)
     {
      last_track = HAL_GetTick();
      track_light();
		 }
       static uint32_t last_oled = 0;
    if (HAL_GetTick() - last_oled >= 50)
    {
	  last_oled = HAL_GetTick();
    char light_str[24];
    sprintf(light_str, "%d %d %d %d      ", 8000-ADC[1], 8000-ADC[3], 8000-ADC[0],8000-ADC[2]);
    OLED_ShowString(0, 57, light_str, 6);
    OLED_Update();
    }
	}
    if (finish)
    {
        uint8_t flag = finish;
			  finish = 0;
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
        if (flag == 1)
				{	
				 dealCMD();
				}
				else if (flag == 2)
    {
        OLED_ShowString(0, 24, "ERR OverLen      ", 6);
        OLED_Update();
		}

		}
//执行完CAL确定MPU_OK后开始更新姿态角
    if(MPU_OK==1) MPU_UpdateAttitude();
   
      pre = cur;
      cur =(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET);

    if (pre != cur)
    {  
			 now=HAL_GetTick();
       if ((now - lastkeytick) > 50) 
       {
				 lastkeytick = now; 
        uint8_t  cur1= (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET) ;
       if (cur1!= cur) { cur = pre; }
			 else
				{
				if (cur == 0)
        {
         t0=HAL_GetTick();       
        }
        else
        {
						t1 =HAL_GetTick();
            uint32_t t= t1- t0;
            if (t >= 1000)
             {
						  cnt=(cnt+2)%3;
						  press=1;
						 }
						else 
						 {
						  cnt=(cnt+1)%3;
						  press=0;
						 }
						   char mode_str[8];
                uint8_t n;
                if (cnt == 0)      
							{ 
								n = 3;
							}
                else if (cnt == 1) 
				{
					n = 1;
					pitch_angle=90,yaw_angle=90;//更新参数，保证下次pitch或yaw指令后未变化的保持90
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,pitch);//PA8
	        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,yaw);//PB3
					OLED_ShowString(0,36,"PIT 90 YAW 90      ",6);//仅更新显示
		      OLED_Update();
			    }
                else if(cnt==2)    n = 2;

                mode_str[0] = 'M';
                mode_str[1] = 'O';
                mode_str[2] = 'D';
                mode_str[3] = 'E';
                mode_str[4] = ' ';
                mode_str[5] = '0' + n;
                mode_str[6] = '\0';

                OLED_ShowString(0,12 , mode_str, 6);
						
						 if (press==1)
                    OLED_ShowString(48, 12, "KEY L", 6);
                else if(press==0)
                    OLED_ShowString(48, 12, "KEY S", 6);

                OLED_Update();
				}
			}
		}
	}
   

		if(cnt==1){HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);}
		else if(cnt==2)
    {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
			
		}
		else if (cnt==0)
		{
							if(HAL_GetTick()-lasttime>=500)
						{
							HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
						  lasttime = HAL_GetTick();
						}
		}
		
        if(MPU_OK==0)
					{
						if(HAL_GetTick()-mpulasttime>=500)
						{
							HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
						  mpulasttime = HAL_GetTick();
						}
					}
					else {HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);}
	
	

    
}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
 
	

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
