#pragma once
#include "../Thrust/ControlSystem.h"
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChBody.h"
#include "MotionCommand.h"
#include "TrajectoryCommand.h"
#include "Course.h"
#include "../../Model/RocketModel.h"
using namespace chrono;
class MotionControlSystem
{
	ControlSystem controlSystem;
	Course course;
	double lookahead;
public:
	MotionControlSystem(ControlSystem controlSystem, Course course, double lookahead);
	TrajectoryCommand getNextTrajectoryCommand(ChVector<> currentPosition, ChVector<> currentVelocity);
	MotionCommand getNextMotionCommand(ChVector<> g_location, RocketModel rocket, double currentTime);
};

