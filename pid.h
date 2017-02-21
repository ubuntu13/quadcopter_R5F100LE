#ifndef PID_H
#define PID_H

#include "r_cg_macrodriver.h"
#include "r_cg_serial.h"

// Examples for _filter:
// f_cut = 10 Hz -> _filter = 15.9155e-3
// f_cut = 15 Hz -> _filter = 10.6103e-3
// f_cut = 20 Hz -> _filter =  7.9577e-3
// f_cut = 25 Hz -> _filter =  6.3662e-3
// f_cut = 30 Hz -> _filter =  5.3052e-3
#define PID_D_TERM_FILTER 0.00795770f    // 20hz filter on D term
//#define PID_D_TERM_FILTER 0.0063662f    // 30hz filter on D term
//#define PID_D_TERM_FILTER 0.0f    // no filter on D term

typedef struct
{
	float kp;
	float ki;
	float kd;
	float desired;
	float prevError;
	float error;
	float integ;
	float deri;	//derivative
	float prevDeri;
	float outP;
	float outI;
	float outD;
	float iLimit;
	float iLimitLow;
	float dt;
}PidObject;

void pidInit(PidObject* pid, const float desired, const float kp,const float ki, const float kd, const float iLimit, const float dt);
float get_p(PidObject* pid);
float get_i(PidObject* pid);
float get_d(PidObject* pid);
float get_pid(PidObject* pid);
void pidReset(PidObject* pid);
void pidSetError(PidObject* pid, const float error);
void pidSetDesired(PidObject* pid, const float desired);

#endif