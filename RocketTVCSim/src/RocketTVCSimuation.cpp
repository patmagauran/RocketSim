#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "Model/RocketModel.h"

#include "Control/Thrust/ThrustParameters.h"

#include "Simulator.h"
#include "Control/Thrust/TunableControlSystem.h"

#include "Control/Thrust/PIDControl/PIDControlSystem.h"

#include "Control/Thrust/PIDControl/TunablePIDControlSystem.h"
#include "Control/Trajectory/LookaheadSystem/LookaheadMotionControlSystem.h"
// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    // Set path to Chrono data directory
    SetChronoDataPath(CHRONO_DATA_DIR);
	RocketParams rocketParams = RocketParams(1, 2, 8, 1, degreesToRad(40), degreesToRad(20), 5);

    // Create a Chrono physical system
	PIDParams pidParamsThrustAngleFromRate = PIDParams(0.0149925, 0.881914, 6.37183e-05, 0.01, rocketParams.getMaxThrustAngle());
	PIDParams pidParamsRateFromAngle = PIDParams(0.0328315, 0.820787, 0.000328315, 0.01, rocketParams.getMaxRotationRate());
	DataLog::initialize("data.csv");



	Course course = Course("C:\\Users\\patma\\source\\repos\\RocketSim\\RocketSimTemplate\\RocketTVCSim\\points.csv");

	//Setup Motion Controller

	std::shared_ptr<TunableControlSystem> tunableControlSystem = std::make_shared<TunablePIDControlSystem>(pidParamsThrustAngleFromRate, pidParamsRateFromAngle);
	//std::shared_ptr<ControlSystem> tunableControlSystem = std::make_shared<PIDControlSystem>(pidParamsRate, pidParamsAngle);
	std::shared_ptr < MotionControlSystem> motionController = std::make_shared <LookaheadMotionControlSystem>(tunableControlSystem, course, 25);
    Simulator sim = Simulator();
	sim.setRocketParams(rocketParams);
	sim.setMotionControlSystem(motionController);
    sim.runSimulation(true);
	sim.cleanup();

	DataLog::initialize("data2.csv");
	tunableControlSystem = std::make_shared<TunablePIDControlSystem>(pidParamsThrustAngleFromRate, pidParamsRateFromAngle, "tyreus-luyben");
	//std::shared_ptr<ControlSystem> tunableControlSystem = std::make_shared<PIDControlSystem>(pidParamsRate, pidParamsAngle);
	motionController = std::make_shared < LookaheadMotionControlSystem>(tunableControlSystem, course, 25);
	sim.setRocketParams(rocketParams);
	sim.setMotionControlSystem(motionController);
	sim.runSimulation(true);
	sim.cleanup();
    return 0;
}
