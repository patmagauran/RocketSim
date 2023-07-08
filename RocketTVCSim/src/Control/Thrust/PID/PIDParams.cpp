#include "PIDParams.h"

PIDParams::PIDParams(double kp, double ki, double kd, double sampleTime, double maxOutput): kp(kp), ki(ki), kd(kd), sampleTime(sampleTime), maxOutput(maxOutput)
{
}

double PIDParams::getKp()
{
	return this->kp;
}

double PIDParams::getKi()
{
	return this->ki;
}

double PIDParams::getKd()
{
	return this->kd;
}

double PIDParams::getSampleTime()
{
	return this->sampleTime;
}

double PIDParams::getMaxOutput()
{
	return this->maxOutput;
}
