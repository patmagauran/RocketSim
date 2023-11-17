#pragma once
#include "../ControlSystem.h"
#include "../pid/PIDParams.h"
#include "../pid/PIDNew.h"
#include "chrono/physics/ChSystemNSC.h"

class PIDControlSystem :
    public virtual ControlSystem
{
protected:
	PIDParams paramsThrustAngleFromRate, paramsRateFromAngle;
	PIDNew yawRateFromAnglePID, yawThrustAngleFromRatePID, pitchRateFromAnglePID, pitchThrustAngleFromRatePID;
public:
	PIDControlSystem(PIDParams paramsThrustAngleFromRate, PIDParams paramsRateFromAngle);

	// Inherited via ControlSystem
	void setParamsThrustAngleFromRate(PIDParams params);

	ControlSystemType getControlSystemType() override;

	void setParamsRateFromAngle(PIDParams params);

	PIDParams getParamsThrustAngleFromRate();

	PIDParams getParamsRateFromAngle();

	double getYawRateFromAngleDeviation(double target, double current, double currentTime);

	double getYawThrustAngleFromRateDeviation(double target, double current, double currentTime);

	double getPitchRateFromAngleDeviation(double target, double current, double currentTime);

	double getPitchThrustAngleFromRateDeviation(double target, double current, double currentTime);
	ThrustParameters computeThrustParameters(ControlState currentState, TrajectoryCommand command, double currentTime) override;
};

