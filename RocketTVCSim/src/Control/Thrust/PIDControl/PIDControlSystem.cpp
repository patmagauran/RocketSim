#include "PIDControlSystem.h"

PIDControlSystem::PIDControlSystem(PIDParams paramsThrustAngleFromRate, PIDParams paramsRateFromAngle) :
    paramsThrustAngleFromRate(paramsThrustAngleFromRate), paramsRateFromAngle(paramsRateFromAngle),
    yawRateFromAnglePID(paramsRateFromAngle.getKd(), paramsRateFromAngle.getKi(), paramsRateFromAngle.getKp(), paramsRateFromAngle.getSampleTime(), paramsRateFromAngle.getMaxOutput()),
    yawThrustAngleFromRatePID(paramsThrustAngleFromRate.getKd(), paramsThrustAngleFromRate.getKi(), paramsThrustAngleFromRate.getKp(), paramsThrustAngleFromRate.getSampleTime(), paramsThrustAngleFromRate.getMaxOutput()),
    pitchRateFromAnglePID(paramsRateFromAngle.getKd(), paramsRateFromAngle.getKi(), paramsRateFromAngle.getKp(), paramsRateFromAngle.getSampleTime(), paramsRateFromAngle.getMaxOutput()),
    pitchThrustAngleFromRatePID(paramsThrustAngleFromRate.getKd(), paramsThrustAngleFromRate.getKi(), paramsThrustAngleFromRate.getKp(), paramsThrustAngleFromRate.getSampleTime(), paramsThrustAngleFromRate.getMaxOutput())
{

}


void PIDControlSystem::setParamsThrustAngleFromRate(PIDParams params)
{
    this->paramsThrustAngleFromRate = params;
    this->yawThrustAngleFromRatePID = PIDNew(params.getKp(), params.getKi(), params.getKd(), params.getSampleTime(), params.getMaxOutput());
    this->pitchThrustAngleFromRatePID = PIDNew(params.getKp(), params.getKi(), params.getKd(), params.getSampleTime(), params.getMaxOutput());
}

void PIDControlSystem::setParamsRateFromAngle(PIDParams params)
{
    this->paramsRateFromAngle = params;
    this->yawRateFromAnglePID = PIDNew(params.getKp(), params.getKi(), params.getKd(), params.getSampleTime(), params.getMaxOutput());
    this->pitchRateFromAnglePID = PIDNew(params.getKp(), params.getKi(), params.getKd(), params.getSampleTime(), params.getMaxOutput());
}

PIDParams PIDControlSystem::getParamsThrustAngleFromRate()
{
    return this->paramsThrustAngleFromRate;
}

PIDParams PIDControlSystem::getParamsRateFromAngle()
{
    return this->paramsRateFromAngle;
}

double PIDControlSystem::getYawRateFromAngleDeviation(double target, double current, double currentTime)
{
    yawRateFromAnglePID.setSetpoint(target);
    return yawRateFromAnglePID.update(current, currentTime);
}

double PIDControlSystem::getYawThrustAngleFromRateDeviation(double target, double current, double currentTime)
{
    yawThrustAngleFromRatePID.setSetpoint(target);
    return yawThrustAngleFromRatePID.update(current, currentTime);
}

double PIDControlSystem::getPitchRateFromAngleDeviation(double target, double current, double currentTime)
{
    pitchRateFromAnglePID.setSetpoint(target);
    return pitchRateFromAnglePID.update(current, currentTime);
}

double PIDControlSystem::getPitchThrustAngleFromRateDeviation(double target, double current, double currentTime)
{
    pitchThrustAngleFromRatePID.setSetpoint(target);
    return pitchThrustAngleFromRatePID.update(current, currentTime);
}
