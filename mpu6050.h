#ifndef MPU6050_H
#define MPU6050_H

#include "r_cg_macrodriver.h"
#include "r_cg_serial.h"

extern uint8_t IICA0_Receiveend;
extern uint8_t IICA0_Sendend;

struct MPU
{
	int16_t GX;
	int16_t GY;
	int16_t GZ;
	int16_t AX;
	int16_t AY;
	int16_t AZ;
};
extern struct MPU MPU_Data;

struct int16_t_XYZ
{
	int16_t X;
	int16_t Y;
	int16_t Z;
};

struct int32_t_XYZ
{
	int32_t X;
	int32_t Y;
	int32_t Z;
};

struct F_XYZ
{
	float X;
	float Y;
	float Z;
};

#define 	SMPLRT_DIV     	0X19
#define 	CONFIG        	0X1A
#define 	GYRO_CONFIG   	0X1B
#define 	ACCEL_CONFIG   	0X1C

#define 	ACCEL_XOUT_H   	0X3B
#define 	ACCEL_XOUT_L   	0X3C
#define 	ACCEL_YOUT_H   	0X3D
#define	 	ACCEL_YOUT_L   	0X3E
#define 	ACCEL_ZOUT_H   	0X3F
#define	    ACCEL_ZOUT_L   	0X40
#define	 	GYRO_XOUT_H    	0X43
#define 	GYRO_XOUT_L    	0X44
#define	 	GYRO_YOUT_H    	0X45
#define 	GYRO_YOUT_L    	0X46
#define 	GYRO_ZOUT_H    	0X47
#define 	GYRO_ZOUT_L    	0X48

#define 	PWR_MGMT_1     	0X6B
#define 	PWR_MGMT_2     	0X6C
#define 	WHO_AM_I       	0X75
#define 	MPU6050_SADR   	0XD0

void MPU6050_Init(void);
uint8_t MPU6050_Read_Byte(uint8_t Reg_Adr);
void MPU6050_Write_Byte(uint8_t Reg_Adr, uint8_t Send_Data);
void MPU6050_Data_Update(void);

#endif