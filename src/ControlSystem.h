#pragma once
#include "PIDParams.h"
#include "PID.h"
class ControlSystem
{
protected:
	PIDParams paramsRate, paramsAngle;
	PID yawAnglePID, yawRatePID, pitchAnglePID, pitchRatePID;

public:
	ControlSystem(PIDParams rateParams, PIDParams angleParams, float maxDeflection = 0.1);

	void setParamsRate(PIDParams params);
	void setParamsAngle(PIDParams params);

	PIDParams getParamsRate();
	PIDParams getParamsAngle();

	float getYawAngle(float target, float current);
float getYawRate(float target, float current);
float getPitchAngle(float target, float current);
float getPitchRate(float target, float current);
};

