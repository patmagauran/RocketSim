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
	virtual ControlSystemType getControlSystemType() override;
	virtual void setParamsThrustAngleFromRate(PIDParams params) override;
	virtual void setParamsRateFromAngle(PIDParams params) override;
	virtual PIDParams getParamsThrustAngleFromRate() override;
	virtual PIDParams getParamsRateFromAngle() override;
	virtual double getYawRateFromAngleDeviation(double target, double current, double currentTime) override;
	virtual double getYawThrustAngleFromRateDeviation(double target, double current, double currentTime) override;
	virtual double getPitchRateFromAngleDeviation(double target, double current, double currentTime) override;
	virtual double getPitchThrustAngleFromRateDeviation(double target, double current, double currentTime) override;
	static std::shared_ptr<LQRControl> fromOptions(std::array<std::string, NUM_CONTROL_OPTIONS> options, double maxThrust, double maxRotationRate);
	ThrustParameters computeThrustParameters(Eigen::Matrix<double, -1, 1> state, Eigen::Matrix<double, -1, 1> desired_state);


	double computeAngle();
};

