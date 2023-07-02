#pragma once
#include "../Thrust/ControlSystem.h"
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChBody.h"
#include "MotionCommand.h"
using namespace chrono;
class MotionControlSystem
{
	ControlSystem controlSystem;
public:
	MotionControlSystem(ControlSystem controlSystem);
	MotionCommand getNextMotionCommand(ChVector<> g_location, std::shared_ptr<ChBody> rocket_upper);
};

