//#include "ControlSystem.h"
//
//ControlSystem::ControlSystem(PIDParams rateParams, PIDParams angleParams) :
//    paramsRate(rateParams), paramsAngle(angleParams),
//    yawAnglePID(angleParams.getKd(), angleParams.getKi(), angleParams.getKp(), angleParams.getSampleTime(), angleParams.getMaxOutput()),
//yawRatePID(rateParams.getKd(), rateParams.getKi(), rateParams.getKp(), rateParams.getSampleTime(), rateParams.getMaxOutput()),
//pitchAnglePID(angleParams.getKd(), angleParams.getKi(), angleParams.getKp(), angleParams.getSampleTime(), angleParams.getMaxOutput()),
//pitchRatePID(rateParams.getKd(), rateParams.getKi(), rateParams.getKp(), rateParams.getSampleTime(), rateParams.getMaxOutput())
//{
//
//}
//
//
//void ControlSystem::setParamsRate(PIDParams params)
//{
//this->paramsRate = params;
//}
//
//void ControlSystem::setParamsAngle(PIDParams params)
//{
//    this->paramsAngle = params;
//}
//
//PIDParams ControlSystem::getParamsRate()
//{
//    return this->paramsRate;
//}
//
//PIDParams ControlSystem::getParamsAngle()
//{
//    return this->paramsAngle;
//}
//
//double ControlSystem::getYawAngle(double target, double current, double currentTime)
//{
//    yawAnglePID.setSetpoint(target);
//    return yawAnglePID.update(current, currentTime);
//}
//
//double ControlSystem::getYawRate(double target, double current, double currentTime)
//{
//yawRatePID.setSetpoint(target);
//	return yawRatePID.update(current, currentTime);
//}
//
//double ControlSystem::getPitchAngle(double target, double current, double currentTime)
//{
//    pitchAnglePID.setSetpoint(target);
//return pitchAnglePID.update(current, currentTime);
//}
//
//double ControlSystem::getPitchRate(double target, double current, double currentTime)
//{
//pitchRatePID.setSetpoint(target);
//	return pitchRatePID.update(current, currentTime);
//}
