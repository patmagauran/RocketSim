
#pragma once
#include "../ControlSystem.h"
#include "../pid/PIDParams.h"
#include "../pid/PIDNew.h"
#include "chrono/physics/ChSystemNSC.h"
#include "../../../Model/RocketParams.h"
class FeedForwardControl :
	public virtual ControlSystem
{
private:
	double lookaheadDistance = 25;
	RocketParams rocketParams;
public:
	FeedForwardControl(RocketParams rocketParams);
	ControlSystemType getControlSystemType() override;
	// Inherited via ControlSystem
	void setParamsThrustAngleFromRate(PIDParams params) override;

	void setParamsRateFromAngle(PIDParams params) override;

	PIDParams getParamsThrustAngleFromRate() override;

	PIDParams getParamsRateFromAngle() override;

	double getYawRateFromAngleDeviation(double target, double current, double currentTime) override;

	double getYawThrustAngleFromRateDeviation(double target, double current, double currentTime) override;

	double getPitchRateFromAngleDeviation(double target, double current, double currentTime) override;

	double getPitchThrustAngleFromRateDeviation(double target, double current, double currentTime) override;
	double ComputeAngle(double velocity, double theta, double ang_velocity);
	static std::shared_ptr<FeedForwardControl> fromOptions(std::array<std::string, NUM_CONTROL_OPTIONS> options, RocketParams rocketParams);
};


