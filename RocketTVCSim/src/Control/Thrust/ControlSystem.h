#pragma once
#include "pid/PIDParams.h"
#include "pid/PIDNew.h"
#include "chrono/physics/ChSystemNSC.h"

enum class ControlSystemType {
	FEED_FORWARD,
	PID,
	LQR,
	MPC
};

class ControlSystem
{
public:
	virtual ControlSystemType getControlSystemType() = 0;
	virtual void setParamsThrustAngleFromRate(PIDParams params) = 0;
	virtual void setParamsRateFromAngle(PIDParams params) = 0;

	virtual PIDParams getParamsThrustAngleFromRate() = 0;
	virtual PIDParams getParamsRateFromAngle() = 0;

	virtual double getYawRateFromAngleDeviation(double target, double current, double currentTime) = 0;
	virtual double getYawThrustAngleFromRateDeviation(double target, double current, double currentTime) = 0;
	virtual double getPitchRateFromAngleDeviation(double target, double current, double currentTime) = 0;
	virtual double getPitchThrustAngleFromRateDeviation(double target, double current, double currentTime) = 0;
};

