#pragma once
#include "RocketModel.h"
class ControlState
{
public:
	double yawAngle = 0; //also rotation.z
	double pitchAngle = 0;//also rotation.x
	double wvelx = 0; //Pitch Rate
	double wvely = 0;
	double wvelz = 0; //yaw Rate
	double velocity = 0;
	double accx = 0;
	double accy = 0;
	double accz = 0;
	double waccy = 0;
	double waccz = 0;
	double eulerdty = 0;
	double eulerdtz = 0;
	double maxThrust = 0;

	Eigen::Matrix<double, 1, -1> convertToSpaceStateState();

	static ControlState computeCurrentStateFromModel(ChVector<> g_location, RocketModel rocket);

	static ControlState computeDesiredState(double yawAngle, double pitchAngle);
};

