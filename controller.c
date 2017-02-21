#include "controller.h"
#include "imu.h"

PidObject pidRollRate;
PidObject pidPitchRate;
PidObject pidYawRate;
PidObject pidAltRate;
PidObject pidRoll;
PidObject pidPitch;
PidObject pidYaw;
PidObject pidAlt;
PidObject pidPosPit;
PidObject pidPosRol;

float rollOutput;
float pitchOutput;
float yawOutput;

extern float altitude_rate;

void controller_init(void)
{
	pidInit(&pidRollRate, 0, 0.10, 0.07, 0.004, 50, controller3AxisUpdate_dt);
  	pidInit(&pidPitchRate, 0, 0.10, 0.07, 0.004, 50, controller3AxisUpdate_dt);
  	pidInit(&pidYawRate, 0, 0.4, 0.05, 0, 10, controller3AxisUpdate_dt);
	pidInit(&pidAltRate, 0, 5, 3, 0, 100, controllerAlt_Update_dt);
	
	
  	pidInit(&pidRoll, 0, 4.0, 0, 0, 0, controller3AxisUpdate_dt);
  	pidInit(&pidPitch, 0, 4.0, 0, 0, 0, controller3AxisUpdate_dt);
  	pidInit(&pidYaw, 0, 4.0, 0, 0, 0, controller3AxisUpdate_dt);
	pidInit(&pidAlt, 40, 1, 0, 0, 0, controllerAlt_Update_dt);
	pidInit(&pidPosPit, 5, 0.10, 0, 0.1, 0, controllerPos_Update_dt);
	//pidInit(&pidPosRol, 0, 0.1, 0, 0.1, 0, controllerPos_Update_dt);
}

void controller_3Axis_AttitudePID(
	float eulerRollActual, float eulerPitchActual, float eulerYawActual,
	float rollRateActual, float pitchRateActual, float yawRateActual)
{
	float yawError;
	
	// Update PID for roll axis attitude
	pidRoll.error = (pidRoll.desired - eulerRollActual) * 10;
  	pidRollRate.desired = get_p(&pidRoll);
	
  	// Update PID for pitch axis attitude
	pidPitch.error = (pidPitch.desired - eulerPitchActual) * 10;
  	pidPitchRate.desired = get_p(&pidPitch);
	
  	// Update PID for yaw axis attitude
  	pidYaw.error = pidYaw.desired - eulerYawActual;
  	if (pidYaw.error > 180.0)
    	pidYaw.error -= 360.0;
 	else if (pidYaw.error < -180.0)
    	pidYaw.error += 360.0;
	pidYaw.error *= 10;
  	pidYawRate.desired = pidYaw.kp * pidYaw.error;
	
	// Update PID for 3 axis Rate
	pidRollRate.error = pidRollRate.desired - rollRateActual * 10;
	pidPitchRate.error = pidPitchRate.desired - pitchRateActual * 10;
	pidYawRate.error = pidYawRate.desired - yawRateActual * 10;
	rollOutput = get_pid(&pidRollRate);
    pitchOutput = get_pid(&pidPitchRate);
    yawOutput = get_pid(&pidYawRate);
}

void controllerResetAllPID(void)
{
  pidReset(&pidRoll);
  pidReset(&pidPitch);
  pidReset(&pidYaw);
  pidReset(&pidAlt);
  pidReset(&pidPosPit);
  pidReset(&pidPosRol);
  pidReset(&pidRollRate);
  pidReset(&pidPitchRate);
  pidReset(&pidYawRate);
  pidReset(&pidAltRate);
}

void controllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw)
{
  *roll = (int16_t)rollOutput;
  *pitch = (int16_t)pitchOutput;
  *yaw = (int16_t)yawOutput;
}

int16_t controllerAltHoldPID(float Actual)
{
	int16_t output = 0;
	static float prevActual;
	float temp;
	
	Actual *= 0.01; //*100cm to *1cm
	temp = Actual - prevActual;
	if( (temp < 0.4 && temp > -0.4) || (temp > 10))
	{
		Actual = prevActual;
	}
	Actual = Actual * 0.8 + prevActual * 0.2;
	altitude_rate = (Actual - prevActual) / 0.025; //get altitude speed
	
	pidAlt.error = Actual - pidAlt.desired;
	pidAltRate.desired = -get_p(&pidAlt); //get altitude desired speed
	
	if(pidAltRate.desired > RisingSpeedLimit)
	{
		pidAltRate.desired = RisingSpeedLimit;
	}
	else if(pidAltRate.desired < -FallingSpeedLimit)
	{
		pidAltRate.desired = -FallingSpeedLimit;
	}

	pidAltRate.error = altitude_rate - pidAltRate.desired;
	output = -(int16_t)get_pid(&pidAltRate);
	
	if(output > AltOutputLimit)
	{
		output = AltOutputLimit;
	}
	else if(output < -AltOutputLimit)
	{
		output = -AltOutputLimit;
	}
	
	prevActual = Actual;
	
	return output;
}

float controllerPosPitHoldPID(float Actual)
{
	static float prevActual;
	float output = 0;
	float temp;
	
	temp = Actual - prevActual;
	if( temp < 0.3 && temp > -0.3 )
	{
		Actual = prevActual;
	}
	//Actual = Actual * 0.8 + prevActual * 0.2;
	
	pidPosPit.error = Actual - pidPosPit.desired;
	output = -get_pid(&pidPosPit);
	
	if(output > PosOutputLimit)
	{
		output = PosOutputLimit;
	}
	else if(output < -PosOutputLimit)
	{
		output = -PosOutputLimit;
	}
	
	prevActual = Actual;
	
	return output;
}

float controllerPosRolHoldPID(float Actual)
{
	static float prevActual;
	float output = 0;
	float temp;
	
	temp = Actual - prevActual;
	if( temp < 0.3 && temp > -0.3 )
	{
		Actual = prevActual;
	}
	Actual = Actual * 0.8 + prevActual * 0.2;
	
	pidPosRol.error = Actual - pidPosRol.desired;
	output = -get_pid(&pidPosRol);
	
	if(output > PosOutputLimit)
	{
		output = PosOutputLimit;
	}
	else if(output < -PosOutputLimit)
	{
		output = -PosOutputLimit;
	}
	
	prevActual = Actual;
	
	return output;
}

int16_t MOTOR_Update(int16_t Motor)
{
	if(Motor > 2000)
	{
		Motor = 2000;
	}
	if(Motor < 1000)
	{
		Motor = 1000;
	}
	
	return Motor;
}









