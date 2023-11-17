#pragma once
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "Model/RocketModelWithDrag.h"

#include "Control/Thrust/ThrustParameters.h"

#include "Control/Trajectory/Course.h"

#include "Control/Thrust/ControlSystem.h"


#include "Control/Trajectory/MotionCommand.h"

#include "Control/Trajectory/MotionControlSystem.h"



#include "Control/Thrust/PID/PIDParams.h"

#include <matplot/matplot.h>
#include "Util/DataLog/DataLog.h"

// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

class TunableControlSystem;
class Simulator
{
private:
	ChSystemNSC sys;
	ThrustParameters thrustParameters;
	RocketModelWithDrag rocket;
	RocketParams rocketParams;
	//std::shared_ptr<TunableControlSystem> tunableControlSystem;
	std::shared_ptr<MotionControlSystem> motionController;
	double maxDeviationFromCourse;
	ChVisualSystemIrrlicht vis;
	bool initialized = false;
public:
	ChSystemNSC* getSystem();
	RocketModel* getRocket();
	void resetSimulator();
	void runSimulation(bool autoTune);
	void cleanup();
	Simulator();
	//void setTunableControlSystem(std::shared_ptr<TunableControlSystem> tunableControlSystem);
	void setMotionControlSystem(std::shared_ptr<MotionControlSystem> motionControlSystem);
	void setRocketParams(RocketParams rocketParams);
	void setMaxDeviationFromCourse(double maxDeviationFromCourse);
	void initialize();
};

//