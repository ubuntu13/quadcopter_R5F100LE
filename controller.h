#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "r_cg_macrodriver.h"
#include "pid.h"

#define controller3AxisUpdate_dt 0.004
#define controllerAlt_Update_dt 0.010
#define controllerPos_Update_dt 0.200

#define RisingSpeedLimit 15 //rising speed limit at 25cm/s
#define FallingSpeedLimit 40 //falling speed limit at 40cm/s
#define AltOutputLimit 300 //altitude output limit at 30% power

#define PosOutputLimit 3 //Loiter output limit at 3 degree

extern PidObject pidRollRate;
extern PidObject pidPitchRate;
extern PidObject pidYawRate;
extern PidObject pidAltRate;
extern PidObject pidRoll;
extern PidObject pidPitch;
extern PidObject pidYaw;
extern PidObject pidAlt;
extern PidObject pidPosPit;
extern PidObject pidPosRol;

extern float loiter_rate;

void controller_init(void);

void controller_3Axis_AttitudePID(
	float eulerRollActual, float eulerPitchActual, float eulerYawActual,
	float rollRateActual, float pitchRateActual, float yawRateActual);
	
int16_t controllerAltHoldPID(float Actual);
float controllerPosPitHoldPID(float Actual);
float controllerPosRolHoldPID(float Actual);
void controllerResetAllPID(void);
void controllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw);
int16_t MOTOR_Update(int16_t Motor);

#endif