#pragma once
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "Model/RocketModel.h"

#include "Control/Thrust/ThrustParameters.h"

#include "Control/Trajectory/Course.h"

#include "Control/Thrust/ControlSystem.h"

#include "Control/Thrust/ControlSystemTuner.h"

#include "Control/Trajectory/MotionCommand.h"

#include "Control/Trajectory/MotionControlSystem.h"

#include "Control/Thrust/TunableControlSystem.h"

#include "Control/Thrust/PID/PIDParams.h"
// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::irrlicht;
class Simulator
{
public:
	void runSimulation();
	Simulator();
};
