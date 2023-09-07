#pragma once
#include "../ControlSystem.h"
#include "../pid/PIDParams.h"
#include "../pid/PIDNew.h"
#include "chrono/physics/ChSystemNSC.h"

class PIDControlSystem :
    public virtual ControlSystem
{
protected:
	PIDParams paramsRate, paramsAngle;
	PIDNew yawAnglePID, yawRatePID, pitchAnglePID, pitchRatePID;
public:
	PIDControlSystem(PIDParams rateParams, PIDParams angleParams);
	// Inherited via ControlSystem
	void setParamsRate(PIDParams params) override;

	void setParamsAngle(PIDParams params) override;

	PIDParams getParamsRate() override;

	PIDParams getParamsAngle() override;

	double getYawAngle(double target, double current, double currentTime) override;

	double getYawRate(double target, double current, double currentTime) override;

	double getPitchAngle(double target, double current, double currentTime) override;

	double getPitchRate(double target, double current, double currentTime) override;
};

