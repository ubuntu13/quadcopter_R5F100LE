#ifndef CAMERA_H
#define CAMERA_H

#include "r_cg_macrodriver.h"
#include "r_cg_serial.h"
#include "imu.h"

#define camerapitch 6.0f//9.4126506f
#define camerapitconvert 0.0025f

typedef struct
{
	float position_x;
	float position_y;
	int16_t camera_x;
	uint16_t camera_y;
	uint8_t erzhi;
	uint8_t sine;
}CameraObject;

extern CameraObject camera;

void camera_process(float roll, float pitch, float alt);

#endif