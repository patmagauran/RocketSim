#pragma once
#include "../ControlSystem.h"
#include "../../../Util/defines.h"
#include <Eigen/Dense>

class LQRController;
class RocketModel;
class ThrustParameters;
class LQRControl :


    public ControlSystem
{
private:
	std::shared_ptr< LQRController> lqrController;
	Eigen::MatrixXd weightMatrixQ;
	Eigen::MatrixXd weightMatrixR;
	int maxIterations = 50;
	double tolerance = 0.01;
	bool initialized = false;
public:
	LQRControl(int maxIterations, double tolerance, Eigen::MatrixXd weightMatrixQ, Eigen::MatrixXd weightMatrixR);
	void initialize(RocketModel model);
	bool getInitialized();
	double getYawRateFromAngleDeviation(double target, double current, double currentTime);


	double getPitchRateFromAngleDeviation(double target, double current, double currentTime);
	virtual ControlSystemType getControlSystemType() override;
	static std::shared_ptr<LQRControl> fromOptions(std::array<std::string, NUM_CONTROL_OPTIONS> options, double maxThrust, double maxRotationRate);
	ThrustParameters computeThrustParameters(Eigen::Matrix<double, -1, 1> state, Eigen::Matrix<double, -1, 1> desired_state);
	ThrustParameters computeThrustParameters(ControlState currentState, TrajectoryCommand command, double currentTime) override;

	double computeAngle();
};

