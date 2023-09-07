#pragma once
#include "pid/PIDParams.h"
#include "pid/PIDNew.h"
#include "chrono/physics/ChSystemNSC.h"

class ControlSystem
{
public:

	virtual void setParamsRate(PIDParams params) = 0;
	virtual void setParamsAngle(PIDParams params) = 0;

	virtual PIDParams getParamsRate() = 0;
	virtual PIDParams getParamsAngle() = 0;

	virtual double getYawAngle(double target, double current, double currentTime) = 0;
	virtual double getYawRate(double target, double current, double currentTime) = 0;
	virtual double getPitchAngle(double target, double current, double currentTime) = 0;
	virtual double getPitchRate(double target, double current, double currentTime) = 0;
};

