
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
	double getYawRateFromAngleDeviation(double target, double current, double currentTime);


	double getPitchRateFromAngleDeviation(double target, double current, double currentTime);
	static std::shared_ptr<MPCControl> fromOptions(std::array<std::string, NUM_CONTROL_OPTIONS> options, double maxThrust, double maxRotationRate);
	ThrustParameters computeThrustParameters(Eigen::Matrix<double, -1, 1> state, Eigen::Matrix<double, -1, 1> desired_state);
	ThrustParameters computeThrustParameters(ControlState currentState, TrajectoryCommand command, double currentTime) override;
	double computeAngle();
};


