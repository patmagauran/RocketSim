#include "PIDParams.h"

PIDParams::PIDParams(float kp, float ki, float kd, float sampleTime, float maxOutput): kp(kp), ki(ki), kd(kd), sampleTime(sampleTime), maxOutput(maxOutput)
{
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

float PIDParams::getSampleTime()
{
	return this->sampleTime;
}

float PIDParams::getMaxOutput()
{
	return this->maxOutput;
}
