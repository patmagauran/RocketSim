#pragma once
#include "../Thrust/ControlSystem.h"
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChBody.h"
#include "MotionCommand.h"
#include "TrajectoryCommand.h"
#include "Course.h"
using namespace chrono;
class MotionControlSystem
{
	ControlSystem controlSystem;
	Course course;
	float lookahead;
public:
	MotionControlSystem(ControlSystem controlSystem, Course course, float lookahead);
	TrajectoryCommand getNextTrajectoryCommand(ChVector<> currentPosition, ChVector<> currentVelocity);
	MotionCommand getNextMotionCommand(ChVector<> g_location, std::shared_ptr<ChBody> rocket_upper, float currentTime);
};

