#pragma once
#include "../../Thrust/ControlSystem.h"
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChBody.h"
#include "../MotionCommand.h"
#include "../TrajectoryCommand.h"
#include "../Course.h"
#include "../../../Model/RocketModel.h"
#include "../MotionControlSystem.h"
using namespace chrono;
class LookaheadMotionControlSystem :
    public MotionControlSystem
{
protected:
	std::shared_ptr<ControlSystem> controlSystem;
	Course course;
	double lookahead;
	ChVector<> lastGoodPoint;
public:
	LookaheadMotionControlSystem(std::shared_ptr<ControlSystem> controlSystem, Course course, double lookahead);
	TrajectoryCommand getNextTrajectoryCommand(ChVector<> currentPosition, ChVector<> currentVelocity);
	MotionCommand getNextMotionCommand(ChVector<> g_location, RocketModel rocket, double currentTime);
	std::vector <ChVector<>> getWaypoints();
	void tune(Simulator* sim);
	double distanceFromTrajectory(ChVector<> currentPosition);

};

