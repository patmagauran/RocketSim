
#pragma once
#include "../ControlSystem.h"
constexpr int Tnx = 12;
constexpr int Tny = 12;
constexpr int Tnu = 4;
constexpr int Tndu = 4;
constexpr int Tph = 10;
constexpr int Tch = 10;
class MPCControl : public ControlSystem
{
private:

	//mpc::LMPC<Tnx, Tnu, Tndu, Tny, Tph, Tch> lmpc;

public:
	// Inherited via ControlSystem
	MPCControl();
	virtual ControlSystemType getControlSystemType() override;
	virtual void setParamsThrustAngleFromRate(PIDParams params) override;
	virtual void setParamsRateFromAngle(PIDParams params) override;
	virtual PIDParams getParamsThrustAngleFromRate() override;
	virtual PIDParams getParamsRateFromAngle() override;
	virtual double getYawRateFromAngleDeviation(double target, double current, double currentTime) override;
	virtual double getYawThrustAngleFromRateDeviation(double target, double current, double currentTime) override;
	virtual double getPitchRateFromAngleDeviation(double target, double current, double currentTime) override;
	virtual double getPitchThrustAngleFromRateDeviation(double target, double current, double currentTime) override;

	double computeAngle();
};


