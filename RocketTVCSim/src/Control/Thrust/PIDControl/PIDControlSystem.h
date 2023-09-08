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
	void setParamsThrustAngleFromRate(PIDParams params) override;

	void setParamsRateFromAngle(PIDParams params) override;

	PIDParams getParamsThrustAngleFromRate() override;

	PIDParams getParamsRateFromAngle() override;

	double getYawRateFromAngleDeviation(double target, double current, double currentTime) override;

	double getYawThrustAngleFromRateDeviation(double target, double current, double currentTime) override;

	double getPitchRateFromAngleDeviation(double target, double current, double currentTime) override;

	double getPitchThrustAngleFromRateDeviation(double target, double current, double currentTime) override;
};

