#include "ControlSystem.h"

ControlSystem::ControlSystem(PIDParams rateParams, PIDParams angleParams,float maxDeflection) : 
    paramsRate(rateParams), paramsAngle(angleParams),
    yawAnglePID(angleParams.getKd(), angleParams.getKi(), angleParams.getKp(), 0.1, 0, maxDeflection),
yawRatePID(rateParams.getKd(), rateParams.getKi(), rateParams.getKp(), 0.1, 0, maxDeflection),
pitchAnglePID(angleParams.getKd(), angleParams.getKi(), angleParams.getKp(), 0.1, 0, maxDeflection),
pitchRatePID(rateParams.getKd(), rateParams.getKi(), rateParams.getKp(), 0.1, 0, maxDeflection)
{

}


void ControlSystem::setParamsRate(PIDParams params)
{
this->paramsRate = params;
}

void ControlSystem::setParamsAngle(PIDParams params)
{
    this->paramsAngle = params;
}

PIDParams ControlSystem::getParamsRate()
{
    return this->paramsRate;
}

PIDParams ControlSystem::getParamsAngle()
{
    return this->paramsAngle;
}

float ControlSystem::getYawAngle(float target, float current)
{
    yawAnglePID.setSetpoint(target);
    return yawAnglePID.update(current);
}

float ControlSystem::getYawRate(float target, float current)
{
yawRatePID.setSetpoint(target);
	return yawRatePID.update(current);
}

float ControlSystem::getPitchAngle(float target, float current)
{
    pitchAnglePID.setSetpoint(target);
return pitchAnglePID.update(current);
}

float ControlSystem::getPitchRate(float target, float current)
{
pitchRatePID.setSetpoint(target);
	return pitchRatePID.update(current);
}
