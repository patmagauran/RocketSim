#pragma once
#include "chrono/core/ChVector.h"
class TrajectoryCommand
{
public:
	inline TrajectoryCommand() : TrajectoryCommand(0, 0, chrono::ChVector<double>(0, 0, 0)) {}
	inline TrajectoryCommand(double yawAngle, double pitchAngle, chrono::ChVector<double> lookaheadPoint): yawAngle(yawAngle), pitchAngle(pitchAngle), lookaheadPoint(lookaheadPoint) {}
	inline TrajectoryCommand(double yawAngle, double pitchAngle, double x, double y, double z) : yawAngle(yawAngle), pitchAngle(pitchAngle), lookaheadPoint(chrono::ChVector<double>(x, y, z)) {}
	double yawAngle, pitchAngle;
	chrono::ChVector<double> lookaheadPoint;
};

