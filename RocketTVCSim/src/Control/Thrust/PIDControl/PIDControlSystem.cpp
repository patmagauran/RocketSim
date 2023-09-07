#include "PIDControlSystem.h"

PIDControlSystem::PIDControlSystem(PIDParams rateParams, PIDParams angleParams) :
    paramsRate(rateParams), paramsAngle(angleParams),
    yawAnglePID(angleParams.getKd(), angleParams.getKi(), angleParams.getKp(), angleParams.getSampleTime(), angleParams.getMaxOutput()),
    yawRatePID(rateParams.getKd(), rateParams.getKi(), rateParams.getKp(), rateParams.getSampleTime(), rateParams.getMaxOutput()),
    pitchAnglePID(angleParams.getKd(), angleParams.getKi(), angleParams.getKp(), angleParams.getSampleTime(), angleParams.getMaxOutput()),
    pitchRatePID(rateParams.getKd(), rateParams.getKi(), rateParams.getKp(), rateParams.getSampleTime(), rateParams.getMaxOutput())
{

}


void PIDControlSystem::setParamsRate(PIDParams params)
{
    this->paramsRate = params;
}

void PIDControlSystem::setParamsAngle(PIDParams params)
{
    this->paramsAngle = params;
}

PIDParams PIDControlSystem::getParamsRate()
{
    return this->paramsRate;
}

PIDParams PIDControlSystem::getParamsAngle()
{
    return this->paramsAngle;
}

double PIDControlSystem::getYawAngle(double target, double current, double currentTime)
{
    yawAnglePID.setSetpoint(target);
    return yawAnglePID.update(current, currentTime);
}

double PIDControlSystem::getYawRate(double target, double current, double currentTime)
{
    yawRatePID.setSetpoint(target);
    return yawRatePID.update(current, currentTime);
}

double PIDControlSystem::getPitchAngle(double target, double current, double currentTime)
{
    pitchAnglePID.setSetpoint(target);
    return pitchAnglePID.update(current, currentTime);
}

double PIDControlSystem::getPitchRate(double target, double current, double currentTime)
{
    pitchRatePID.setSetpoint(target);
    return pitchRatePID.update(current, currentTime);
}
