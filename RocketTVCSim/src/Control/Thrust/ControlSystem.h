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

	float getYawAngle(float target, float current, float currentTime);
float getYawRate(float target, float current, float currentTime);
float getPitchAngle(float target, float current, float currentTime);
float getPitchRate(float target, float current, float currentTime);
};

