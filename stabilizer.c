#include "stabilizer.h"
#include "controller.h"
#include "imu.h"
#include "mpu6050.h"
#include "com.h"

extern float altitude;

int16_t M_Roll = 0, M_Pitch = 0, M_Yaw = 0, M_Mmr = 1000;
int16_t M_Thr = 1000, M_ALt = 0;
uint8_t HIGH_CRL = 1;
//1: open high control
//0: close high control
uint8_t cnt = 0;

void stabilize_task(void)
{
	if(M_Thr > 1000)  //only fly_mode
	{
		controller_3Axis_AttitudePID(Attitude.Roll, Attitude.Pitch, Attitude.Yaw, MPU_Data.GX * 0.060975, -MPU_Data.GY * 0.060975, -MPU_Data.GZ * 0.060975);
		controllerGetActuatorOutput(&M_Roll, &M_Pitch, &M_Yaw);
		
		if(HIGH_CRL)
		{	//High_Control Enabled
			M_Mmr = M_Thr + M_ALt;
		}
		else
		{	//High_Control Not Enabled
			M_Mmr = M_Thr;
		}
	}
	else
	{	//High_Control Not Enabled
		M_Mmr = M_Thr;
	}
	
	MOTOR1 = MOTOR_Update((M_Mmr + M_Pitch + M_Roll - M_Yaw));
	MOTOR2 = MOTOR_Update((M_Mmr + M_Pitch - M_Roll + M_Yaw));
	MOTOR3 = MOTOR_Update((M_Mmr - M_Pitch - M_Roll - M_Yaw));
	MOTOR4 = MOTOR_Update((M_Mmr - M_Pitch + M_Roll + M_Yaw));
	
	cnt++;
	if(cnt == 2)
	{
		cnt = 0;
		Send_Messge();	//communication
	}
}
