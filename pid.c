#include "pid.h"
#include "r_cg_serial.h"
#include "r_cg_userdefine.h"

void pidInit(PidObject* pid, const float desired, const float kp,const float ki, const float kd, const float iLimit, const float dt)
{
	pid->kp = kp;
  	pid->ki = ki;
  	pid->kd = kd;  	
	pid->desired = desired;
	pid->prevError = 0;
  	pid->error = 0;
	pid->integ = 0;
  	pid->deri = 0;
  	pid->prevDeri = 0;
	pid->outP = 0;
	pid->outI = 0;
	pid->outD = 0;
	pid->iLimit = iLimit;
  	pid->iLimitLow = -iLimit;
	pid->dt = dt;
}

float get_p(PidObject* pid)
{
	pid->outP = pid->kp * pid->error;
	return pid->outP;
}

float get_i(PidObject* pid)
{
	pid->outI += (pid->ki * pid->error * pid->dt);
	if(pid->outI > pid->iLimit)
	{
		pid->outI = pid->iLimit;
	}
	else if(pid->outI < pid->iLimitLow)
	{
		pid->outI = pid->iLimitLow;
	}
	return pid->outI;
}

float get_d(PidObject* pid)
{
	pid->deri = (pid->error - pid->prevError) / pid->dt;	//D
	pid->deri = pid->prevDeri + ( pid->dt / ( PID_D_TERM_FILTER + pid->dt ) ) * ( pid->deri - pid->prevDeri );
	
	pid->prevDeri = pid->deri;
	pid->prevError = pid->error;
	
	pid->outD = pid->kd * pid->deri;
	
	return pid->outD;
}

float get_pid(PidObject* pid)
{
	float output;
	
	pid->outP = pid->kp * pid->error;	//P
	
	pid->outI += pid->ki * pid->error * pid->dt;	//I
	if (pid->outI > pid->iLimit)
    {
        pid->outI = pid->iLimit;
    }
    else if (pid->outI < pid->iLimitLow)
    {
        pid->outI = pid->iLimitLow;
    }
	
	pid->deri = (pid->error - pid->prevError) / pid->dt;	//D
	pid->deri = pid->prevDeri + ( pid->dt / ( PID_D_TERM_FILTER + pid->dt ) ) * ( pid->deri - pid->prevDeri );
	
	pid->prevDeri = pid->deri;
	pid->prevError = pid->error;
	
	pid->outD = pid->kd * pid->deri;
	
	output = pid->outP + pid->outI + pid->outD;
	
	return output;
}

void pidReset(PidObject* pid)
{
  pid->error     = 0;
  pid->prevError = 0;
  pid->integ     = 0;
  pid->deri     = 0;
}

void pidSetError(PidObject* pid, const float error)
{
  pid->error = error;
}

void pidSetDesired(PidObject* pid, const float desired)
{
  pid->desired = desired;
}
