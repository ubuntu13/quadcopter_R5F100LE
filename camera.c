#include "camera.h"

CameraObject camera;
float change_x, change_y;

void camera_process(float roll, float pitch, float alt)
{
    change_x = camera.camera_x + pitch * camerapitch;//go + back -
	camera.position_x = change_x * alt * 0.01 * camerapitconvert;
}