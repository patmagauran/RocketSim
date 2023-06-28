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
// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::irrlicht;
class Simulator
{
public:
	void runSimulation();
	Simulator();
};

