
#pragma once
#include "../ControlSystem.h"
#include "../../../Util/defines.h"
#include <Eigen/Dense>
#include <mpc/LMPC.hpp>

constexpr int Tnx = 7; // State size
constexpr int Tny = 7; // State size
constexpr int Tnu = 3; // Control Size
constexpr int Tndu = 4; // Disturbance space size
constexpr int Tph = 10; // Prediction horizon
constexpr int Tch = 10; // Control horizon
class ThrustParameters;
class RocketModel;

//template <int Tnx = Eigen::Dynamic, int Tnu = Eigen::Dynamic, int Tndu = Eigen::Dynamic,
//	int Tny = Eigen::Dynamic, int Tph = Eigen::Dynamic, int Tch = Eigen::Dynamic> class LMPC;
class MPCControl : public ControlSystem
{
private:

	mpc::LMPC<> optsolver = mpc::LMPC<>(Tnx, Tnu, Tndu, Tny, Tph, Tch);
	bool initialized = false;
	//std::shared_ptr<mpc::LMPC<>> optsolver;

public:
	// Inherited via ControlSystem
	MPCControl();
	void initialize(RocketModel model);
	bool getInitialized();
	virtual ControlSystemType getControlSystemType() override;
	virtual void setParamsThrustAngleFromRate(PIDParams params) override;
	virtual void setParamsRateFromAngle(PIDParams params) override;
	virtual PIDParams getParamsThrustAngleFromRate() override;
	virtual PIDParams getParamsRateFromAngle() override;
	virtual double getYawRateFromAngleDeviation(double target, double current, double currentTime) override;
	virtual double getYawThrustAngleFromRateDeviation(double target, double current, double currentTime) override;
	virtual double getPitchRateFromAngleDeviation(double target, double current, double currentTime) override;
	virtual double getPitchThrustAngleFromRateDeviation(double target, double current, double currentTime) override;
	static std::shared_ptr<MPCControl> fromOptions(std::array<std::string, NUM_CONTROL_OPTIONS> options, double maxThrust, double maxRotationRate);
	ThrustParameters computeThrustParameters(Eigen::Matrix<double, -1, 1> state, Eigen::Matrix<double, -1, 1> desired_state);

	double computeAngle();
};


