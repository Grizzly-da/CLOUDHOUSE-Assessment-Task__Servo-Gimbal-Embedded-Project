/*****************************************************************************
  文件: mpu6050.c
  作者: Zhengyu https://gzwelink.taobao.com
  版本: V1.0.0
  时间: 20200401
	平台:MINI-F103C8T6

*******************************************************************************/

/* 包含头文件 ----------------------------------------------------------------*/
#include "mpu6050.h"
#include "i2c.h"
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/

//函数功能: 写数据到MPU6050寄存器
void MPU6050_WriteReg(uint8_t reg_add,uint8_t reg_dat)
{
	HAL_I2C_Mem_Write(&hi2c2,MPU6050_SLAVE_ADDRESS,reg_add,1,&reg_dat,1,1000);
}

//函数功能: 从MPU6050寄存器读取数据
void MPU6050_ReadData(uint8_t reg_add,unsigned char*Read,uint8_t num)
{
 HAL_I2C_Mem_Read(&hi2c2,MPU6050_SLAVE_ADDRESS,reg_add,1,Read,num,1000);
}


void Motion_Interrupt(void)             //运动中断
{
    MPU6050_WriteReg(MPU6050_RA_MOT_THR,30);              //运动阈值，如果要增加灵敏度，可以把值改到10以下
    MPU6050_WriteReg(MPU6050_RA_MOT_DUR,0x14);            //16进制，检测时间20ms 单位1ms 
}

//函数功能: 中断初始化设置interrupt
void MPU_INT_Init(void)
{

 
//自由落体、运动、静止中断 三选一  
//    Free_Fall_Interrupt();                          //自由落体中断
   // Motion_Interrupt();                               //运动中断
//    Zero_Motion_Interrupt();                        //静止中断
    MPU6050_WriteReg(MPU6050_RA_CONFIG,0x04);           //配置外部引脚采样和DLPF数字低通滤波器
    MPU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG,0x04);     //加速度传感器量程和高通滤波器配置
//    MPU6050_WriteReg(MPU6050_RA_INT_PIN_CFG,0X1C);      //INT引脚低电平平时
//    MPU6050_WriteReg(MPU6050_RA_INT_ENABLE,0x40);       //中断使能寄存器
}
//函数功能: 初始化MPU6050芯片
void MPU6050_Init(void)
{
  int i=0,j=0;
  //在初始化之前要延时一段时间，若没有延时，则断电后再上电数据可能会出错
  for(i=0;i<1000;i++)
  {
    for(j=0;j<1000;j++)
    {
      ;
    }
  }
	MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x00);	    //解除休眠状态
	MPU6050_WriteReg(MPU6050_RA_SMPLRT_DIV , 0x07);	    //陀螺仪采样率，1KHz
	MPU6050_WriteReg(MPU6050_RA_CONFIG , 0x06);	        //低通滤波器的设置，截止频率是1K，带宽是5K
	MPU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG , 0x00);	  //配置加速度传感器工作在2G模式，不自检
	MPU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x18);     //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
	MPU_INT_Init();
	

}

/**
  * 函数功能: 读取MPU6050的ID
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */ 
uint8_t MPU6050ReadID(void)
{
	unsigned char Re = 0;
    MPU6050_ReadData(MPU6050_RA_WHO_AM_I,&Re,1);    //读器件地址
	if(Re != 0x70)
	{
		return 0;
	}
	else
	{
		return 1;
	}
		
}

//函数功能: 读取MPU6050的加速度数据
  
void MPU6050ReadAcc(short *accData)
{
    uint8_t buf[6];
		MPU6050_ReadData(MPU6050_ACC_OUT, &buf[0], 6);

	
    accData[0] = (buf[0] << 8) | buf[1];
    accData[1] = (buf[2] << 8) | buf[3];
    accData[2] = (buf[4] << 8) | buf[5];
}

//函数功能: 读取MPU6050的角速度数据
  
void MPU6050ReadGyro(short *gyroData)
{
    uint8_t buf[6];
    MPU6050_ReadData(MPU6050_GYRO_OUT,buf,6);
    gyroData[0] = (buf[0] << 8) | buf[1];
    gyroData[1] = (buf[2] << 8) | buf[3];
    gyroData[2] = (buf[4] << 8) | buf[5];
}

//读取MPU6050的原始温度数据
 
void MPU6050ReadTemp(short *tempData)
{
	uint8_t buf[2];
	MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);     //读取温度值
	*tempData = (buf[0] << 8) | buf[1];
}

//函数功能: 读取MPU6050的温度数据，转化成摄氏度
void MPU6050_ReturnTemp(short*Temperature)
{
	short temp3;
	uint8_t buf[2];
	
	MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);     //读取温度值
	temp3= (buf[0] << 8) | buf[1];
	*Temperature=(((double) (temp3 + 13200)) / 280)-13;
}

 


