
#include "imu.h"
#include "mpu6050.h"
#include <math.h>

#define 	Buf_Num  	19
#define     RtoA        57.2957795f
#define     AtoR        0.01745329f
#define     Gyrot       0.00106422f
#define 	pitchbuf_num 16

struct Angle Attitude, Expect;
struct F_XYZ Fil_Accel, Fil_Gyro;

int16_t Buf_AX[Buf_Num], Buf_AY[Buf_Num], Buf_AZ[Buf_Num];
float pitch_buffer[pitchbuf_num];

extern uint8_t MPU6050_Data_Buf[14];
extern uint8_t Time_Distence_OK;

uint8_t buffer[26] = 
{
	'G', 'Y'
};

void Send_MPU_Data(void)
{
	uint8_t i = 0;
	
 	buffer[2] = (uint8_t)((uint32_t)Fil_Accel.X>>24);
	buffer[3] = (uint8_t)((uint32_t)Fil_Accel.X>>16);
	buffer[4] = (uint8_t)((uint32_t)Fil_Accel.X>>8);
	buffer[5] = (uint8_t)((uint32_t)Fil_Accel.X);
	buffer[6] = (uint8_t)((uint32_t)Fil_Accel.Y>>24);
	buffer[7] = (uint8_t)((uint32_t)Fil_Accel.Y>>16);
	buffer[8] = (uint8_t)((uint32_t)Fil_Accel.Y>>8);
	buffer[9] = (uint8_t)((uint32_t)Fil_Accel.Y);
	buffer[10] = (uint8_t)((uint32_t)Fil_Accel.Z>>24);
	buffer[11] = (uint8_t)((uint32_t)Fil_Accel.Z>>16);
	buffer[12] = (uint8_t)((uint32_t)Fil_Accel.Z>>8);
	buffer[13] = (uint8_t)((uint32_t)Fil_Accel.Z);
	
	buffer[14] = (uint8_t)((uint32_t)Fil_Gyro.X>>24);
	buffer[15] = (uint8_t)((uint32_t)Fil_Gyro.X>>16);
	buffer[16] = (uint8_t)((uint32_t)Fil_Gyro.X>>8);
	buffer[17] = (uint8_t)((uint32_t)Fil_Gyro.Y);
	buffer[18] = (uint8_t)((uint32_t)Fil_Gyro.Y>>24);
	buffer[19] = (uint8_t)((uint32_t)Fil_Gyro.Y>>16);
	buffer[20] = (uint8_t)((uint32_t)Fil_Gyro.Y>>8);
	buffer[21] = (uint8_t)((uint32_t)Fil_Gyro.Z);
	buffer[22] = (uint8_t)((uint32_t)Fil_Gyro.Z>>24);
	buffer[23] = (uint8_t)((uint32_t)Fil_Gyro.Z>>16);
	buffer[24] = (uint8_t)((uint32_t)Fil_Gyro.Z>>8);
	buffer[25] = (uint8_t)((uint32_t)Fil_Gyro.Z);
	
	R_UART0_Send(buffer, 26);
}

void Filter_Init(void)
{
	Fil_Gyro.X = 0;
	Fil_Gyro.Y = 0;
	Fil_Gyro.Z = 0;
	
	Fil_Accel.X = 0;
	Fil_Accel.Y = 0;
	Fil_Accel.Z = 0;
}

void IMU_Prepare(void)
{
    uint8_t i = 0;
	float temp_X = 0, temp_Y = 0, temp_Z = 0;
	
	float Hn[Buf_Num] =
	{
		-0.0048,  0.0000,  0.0155,  0.0186, -0.0152,
 		-0.0593, -0.0345,  0.1045,  0.2881,  0.3739,
 		 0.2881,  0.1045, -0.0345, -0.0593, -0.0152,
		 0.0186,  0.0155,  0.0000, -0.0048
	};
	
	Fil_Gyro.X = MPU_Data.GX * Gyrot;
	Fil_Gyro.Y = MPU_Data.GY * Gyrot;
	Fil_Gyro.Z = MPU_Data.GZ * Gyrot;
	
	Buf_AX[0] = MPU_Data.AX;
	Buf_AY[0] = MPU_Data.AY;
	Buf_AZ[0] = MPU_Data.AZ;
	
	for(i = 0; i < Buf_Num; i++)
	{
		temp_X += Buf_AX[i] * Hn[i];
		temp_Y += Buf_AY[i] * Hn[i];
		temp_Z += Buf_AZ[i] * Hn[i];
	}
	
	Fil_Accel.X = temp_X;
	Fil_Accel.Y = temp_Y;
	Fil_Accel.Z = temp_Z;
	
	for(i = 0; i < Buf_Num -1; i++)
	{
		Buf_AX[Buf_Num-1-i] = Buf_AX[Buf_Num-2-i];
		Buf_AY[Buf_Num-1-i] = Buf_AY[Buf_Num-2-i];
		Buf_AZ[Buf_Num-1-i] = Buf_AZ[Buf_Num-2-i];
	}
}

void imu_buffer(void)
{
    uint8_t i = 0;
	float temp_pitch = 0;
	
	for(i = 0; i < pitchbuf_num-1; i++)
	{
		pitch_buffer[pitchbuf_num-1-i] = pitch_buffer[pitchbuf_num-2-i];
	}
	pitch_buffer[0] = Attitude.Pitch;
}

float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

float Get_TrueTime(void)
{
	static uint16_t TCR_Now = 0, TCR_Last = 60000;
	float temp;
	
	TCR_Now = TCR06;
	
	if(TCR_Last > TCR_Now)
	{
		temp = (float)( TCR_Last - TCR_Now ) * 0.000001f;
	}
	else
	{
		temp = (float)( 60000 - TCR_Now + TCR_Last ) * 0.000001f;
	}
	
	TCR_Last = TCR_Now;
	
	return temp;
}

#define twoKp 		0.8f    //0.1f 		//0.8f		//2.0f
#define twoKi 		0.002f  //0.001f 	//0.002f	//0.008

float dt = 0.004f;

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float integralFBx = 0, integralFBy = 0, integralFBz = 0;

void IMU_Update(float gx, float gy, float gz, float ax, float ay, float az)
{
  	float recipNorm;
  	float halfvx, halfvy, halfvz;
  	float halfex, halfey, halfez;
	float qa, qb, qc;

  	recipNorm = invSqrt(ax*ax + ay*ay + az*az);
  	ax = ax * recipNorm;
  	ay = ay * recipNorm;
  	az = az * recipNorm;
           
  	halfvx = q1 * q3 - q0 * q2;
	halfvy = q0 * q1 + q2 * q3;
	halfvz = q0 * q0 - 0.5f + q3 * q3;

  	halfex = (ay * halfvz - az * halfvy);
  	halfey = (az * halfvx - ax * halfvz);
  	halfez = (ax * halfvy - ay * halfvx);

	dt = Get_TrueTime();
	
  	integralFBx += twoKi * halfex * dt;  // integral error scaled by Ki
    integralFBy += twoKi * halfey * dt;
    integralFBz += twoKi * halfez * dt;
    gx += integralFBx;  // apply integral feedback
    gy += integralFBy;
    gz += integralFBz;

  	gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
					   
  	gx *= (0.5f * dt);   // pre-multiply common factors
    gy *= (0.5f * dt);
  	gz *= (0.5f * dt);
  	qa = q0;
  	qb = q1;
  	qc = q2;
  	q0 += (-qb * gx - qc * gy - q3 * gz);
  	q1 += (qa * gx + qc * gz - q3 * gy);
  	q2 += (qa * gy - qb * gz + q3 * gx);
  	q3 += (qa * gz + qb * gy - qc * gx);

  	// Normalise quaternion
  	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  	q0 *= recipNorm;
  	q1 *= recipNorm;
  	q2 *= recipNorm;
  	q3 *= recipNorm;
	
  	Attitude.Pitch = RtoA * asinf(2 * q1 * q3 - 2 * q0 * q2);
  	Attitude.Roll  = RtoA * atan2f(2 * q0 * q1 + 2 * q2 * q3, q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3); //2*q2q3 + 2*q0q1, -2*q1q1-2*q2q2 + 1
	Attitude.Yaw  = RtoA * -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1);//MPU_Data.GZ * 0.015267 * 0.004f;     
}

void Filter(void)
{
	IMU_Prepare();
	
	IMU_Update(Fil_Gyro.X, Fil_Gyro.Y, Fil_Gyro.Z, Fil_Accel.X, Fil_Accel.Y, Fil_Accel.Z);
}