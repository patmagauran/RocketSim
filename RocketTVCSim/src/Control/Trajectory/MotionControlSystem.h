#pragma once
#include "chrono/core/ChVector.h"
#include "MotionCommand.h"
#include "TrajectoryCommand.h"
#include "../../Model/RocketModel.h"
using namespace chrono;
class Simulator;
class MotionControlSystem
{
public:
	virtual TrajectoryCommand getNextTrajectoryCommand(ChVector<> currentPosition, ChVector<> currentVelocity) = 0;
	virtual MotionCommand getNextMotionCommand(ChVector<> g_location, RocketModel rocket, double currentTime) = 0;
	virtual std::vector <ChVector<>> getWaypoints() = 0;
	virtual void tune(Simulator* sim) = 0;
	virtual ChVector<> getClosestPoint(ChVector<> currentPosition) = 0;
	virtual 	double getPercentComplete(ChVector<> closestPoint) = 0;
};

