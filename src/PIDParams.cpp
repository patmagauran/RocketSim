#include "PIDParams.h"

PIDParams::PIDParams(float kp, float ki, float kd)
{
this->kp = kp;
	this->ki = ki;
	this->kd = kd;
}

float PIDParams::getKp()
{
	return this->kp;
}

float PIDParams::getKi()
{
	return this->ki;
}

float PIDParams::getKd()
{
	return this->kd;
}
