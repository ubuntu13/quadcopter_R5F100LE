#ifndef IMU_H
#define IMU_H

#include "r_cg_macrodriver.h"
#include "r_cg_serial.h"

#define IMU_UPDATE_DT 0.004

struct Angle
{
	float Pitch;
	float Roll;
	float Yaw;
};
extern struct Angle Attitude;
extern struct Angle Expect;
extern float pitch_buffer[];

void IMU_Prepare(void);
void IMU_Update(float gx, float gy, float gz, float ax, float ay, float az);
void Filter(void);
void Filter_Init(void);
void imu_buffer(void);

#endif