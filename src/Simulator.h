#pragma once
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "RocketModel.h"

#include "ThrustParameters.h"

#include "Course.h"

#include "ControlSystem.h"

#include "ControlSystemTuner.h"

#include "MotionCommand.h"

#include "MotionControlSystem.h"

#include "TunableControlSystem.h"

#include "PIDParams.h"
// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::irrlicht;
class Simulator
{
public:
	void runSimulation();
	Simulator();
};

