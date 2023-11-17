#include "LQRControl.h"
#include <iostream>
#include <vector>

#include<Eigen/Dense>
#include <Eigen/Eigenvalues> 

#include "LQRController.h"
#include "../../../Model/RocketModel.h"
#include "../../Trajectory/TrajectoryCommand.h"
#include "../ThrustParameters.h"
#include "../../../Model/ControlState.h"

using namespace Eigen;
using namespace std;
LQRControl::LQRControl(int maxIterations, double tolerance, MatrixXd weightMatrixQ, MatrixXd weightMatrixR) : maxIterations(maxIterations), tolerance(tolerance), weightMatrixQ(weightMatrixQ), weightMatrixR(weightMatrixR)
{
}


void LQRControl::initialize(RocketModel model)
{

    Matrix <double, -1, -1> Ac = model.getAmatrix();
        Matrix <double, -1, -1> Bc = model.getBmatrix();

   
    // extract the number of rows and columns
    unsigned int n = Ac.rows();
    unsigned int m = Bc.cols();

    // construct the weigthing matrices
    // state weighting matrix


    // mixed state-input weighting matrix
    MatrixXd weightMatrixS;
    weightMatrixS.resize(n, m);
    weightMatrixS.setZero();


    // construct the LQR controller object
    lqrController = std::make_shared<LQRController>(LQRController(Ac, Bc, this->weightMatrixQ, this->weightMatrixR, weightMatrixS));
    lqrController->ComputeSolution(this->maxIterations, this->tolerance);
    initialized = true;
}

ControlSystemType LQRControl::getControlSystemType()
{
	return ControlSystemType::LQR;
}



bool LQRControl::getInitialized()
{
    return initialized;
}

double LQRControl::getYawRateFromAngleDeviation(double target, double current, double currentTime)
{
    return (target - current) / 5;

	return 0.0;
}



double LQRControl::getPitchRateFromAngleDeviation(double target, double current, double currentTime)
{
    return (target - current) / 5;

	return 0.0;
}


std::shared_ptr<LQRControl> LQRControl::fromOptions(std::array<std::string, NUM_CONTROL_OPTIONS> options, double maxThrust, double maxRotationRate)
{
    Eigen::MatrixXd weightMatrixQ;
    weightMatrixQ = 100 * Eigen::MatrixXd::Identity(7, 7);

    // input weighting matrix
    Eigen::MatrixXd weightMatrixR;
    weightMatrixR = 0.01 * Eigen::MatrixXd::Identity(3, 3);
    return std::make_shared<LQRControl>(LQRControl(stod(options[0]), stod(options[1]), weightMatrixQ, weightMatrixR));
}

double LQRControl::computeAngle()
{
	return 0.0;
}

ThrustParameters LQRControl::computeThrustParameters(Matrix<double, -1, 1> state, Matrix<double, -1, 1> desired_state)
{
    
    Matrix<double, -1, 1> optControl = lqrController->computeOptimalControl(state, desired_state, 50);
    double gamma1 = optControl(0, 0);
    double gamma2 = optControl(1, 0);
    double thrust = optControl(2, 0);
    if (std::isnan(gamma1)) {
        gamma1 = 0;
    }
    if (std::isnan(gamma2)) {
		gamma2 = 0;
	}
    if (std::isnan(thrust)) {
        thrust = 1;
    }
    if (thrust < 0) {
		thrust = 0;
	}
    return ThrustParameters(optControl(0,0), optControl(1,0), optControl(2,0));
}

ThrustParameters LQRControl::computeThrustParameters(ControlState currentState, TrajectoryCommand command, double currentTime)
{
    Eigen::Matrix<double, 1, -1> state = currentState.convertToSpaceStateState();
    Eigen::Matrix<double, 1, 7> desiredState = Eigen::Matrix<double, 1, 7>::Zero();
    //Desired acceleration is 0
    //Desired angular acceleration is 0
    //Desired euler velocities are computed from the trajectory command
    double yawAngleO = getYawRateFromAngleDeviation(command.yawAngle, currentState.yawAngle, currentTime);
    double pitchAngleO = getPitchRateFromAngleDeviation(command.pitchAngle, currentState.pitchAngle, currentTime);
    desiredState[5] = yawAngleO;
    desiredState[6] = pitchAngleO;
    ThrustParameters params = computeThrustParameters(state, desiredState);
    return params;
}
