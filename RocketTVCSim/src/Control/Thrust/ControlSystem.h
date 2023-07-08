#pragma once
#include "pid/PIDParams.h"
#include "pid/PIDNew.h"
#include "chrono/physics/ChSystemNSC.h"

class ControlSystem
{
protected:
	PIDParams paramsRate, paramsAngle;
	PIDNew yawAnglePID, yawRatePID, pitchAnglePID, pitchRatePID;
public:
	ControlSystem(PIDParams rateParams, PIDParams angleParams);

	void setParamsRate(PIDParams params);
	void setParamsAngle(PIDParams params);

	PIDParams getParamsRate();
	PIDParams getParamsAngle();

	double getYawAngle(double target, double current, double currentTime);
double getYawRate(double target, double current, double currentTime);
double getPitchAngle(double target, double current, double currentTime);
double getPitchRate(double target, double current, double currentTime);
};

