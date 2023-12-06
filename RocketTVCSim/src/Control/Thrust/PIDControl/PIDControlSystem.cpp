#include "PIDControlSystem.h"
#include "../../Trajectory/TrajectoryCommand.h"
#include "../ThrustParameters.h"
#include "../../../Model/ControlState.h"
#include "../../../Util/DataLog/DataLog.h"
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

ControlSystemType PIDControlSystem::getControlSystemType()
{
    return ControlSystemType::PID;
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
    double yawThrustAngle= yawThrustAngleFromRatePID.update(current, currentTime);
    DataLog::logData("targetYawRate", target);
    DataLog::logData("yawThrustAngle", yawThrustAngle);

    return yawThrustAngle;
}

double PIDControlSystem::getPitchRateFromAngleDeviation(double target, double current, double currentTime)
{
    pitchRateFromAnglePID.setSetpoint(target);
    return pitchRateFromAnglePID.update(current, currentTime);
}

double PIDControlSystem::getPitchThrustAngleFromRateDeviation(double target, double current, double currentTime)
{
    pitchThrustAngleFromRatePID.setSetpoint(target);
    double pitchThrustAngle= pitchThrustAngleFromRatePID.update(current, currentTime);
    DataLog::logData("targetPitchRate", target);
    DataLog::logData("pitchThrustAngle", pitchThrustAngle);
    return pitchThrustAngle;
}

ThrustParameters PIDControlSystem::computeThrustParameters(ControlState currentState, TrajectoryCommand command, double currentTime)
{
    double yawAngleO = getYawRateFromAngleDeviation(-command.yawAngle, currentState.yawAngle, currentTime);

    double yawThrustAng = getYawThrustAngleFromRateDeviation(yawAngleO, currentState.wvelz, currentTime);

    double pitchAngleO = getPitchRateFromAngleDeviation(command.pitchAngle,currentState.pitchAngle, currentTime);

    double pitchThrustAng = getPitchThrustAngleFromRateDeviation(pitchAngleO, currentState.wvelx, currentTime);
    return ThrustParameters(yawThrustAng, pitchThrustAng, currentState.maxThrust);
}
