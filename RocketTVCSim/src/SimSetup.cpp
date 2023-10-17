#include "SimSetup.h"
#include "Control/Thrust/ControlSystem.h"
#include "Control/Thrust/FeedForwardControl/FeedForwardControl.h"
#include "Control/Thrust/PIDControl/TunablePIDControlSystem.h"
#include "Control/Trajectory/LookaheadSystem/LookaheadMotionControlSystem.h"
#include "Control/Thrust/MPCControl/MPCControl.h"
SimSetup::SimSetup(std::shared_ptr<MotionControlSystem> motionControlSystem, RocketParams rocketParams) : motionControlSystem(motionControlSystem), rocketParams(rocketParams)
{
}

SimSetup SimSetup::fromCLIFlags(int argc, char* argv[])
{
	return fromParameters(SimParams::fromCLIFlags(argc, argv));
}

SimSetup SimSetup::fromCSVRow(std::vector<std::string> csvRow, int controlSysCol, int motionControlCol, int rocketModelCol)
{

	return fromParameters(SimParams::fromCSVRow(csvRow, controlSysCol, motionControlCol, rocketModelCol));
}

SimSetup SimSetup::fromParameters(SimParams simParams)
{
	std::shared_ptr<MotionControlSystem> motionControlSystem;
	std::shared_ptr<ControlSystem> controlSystem;
	RocketParams rocketParams;
	MPCControl mpcControl;
	mpcControl.computeAngle();
	if (simParams.getRocketModel() == "Simple") {
		std::array<std::string, NUM_ROCKET_PARAMS> rocketOptions = simParams.getRocketOptions();
		rocketOptions[7] = "0"; // Set drag to 0
		rocketParams = RocketParams::fromOptions(simParams.getRocketOptions());
	}
	else if (simParams.getRocketModel() == "drag") {
		rocketParams = RocketParams::fromOptions(simParams.getRocketOptions());

	}
	else {
		throw std::invalid_argument("Invalid Rocket Model");
	}
	if (simParams.getControlSystem() == "PID") {
		controlSystem = TunablePIDControlSystem::fromOptions(simParams.getControlOptions(), rocketParams.getMaxThrustAngle(), rocketParams.getMaxRotationRate());
	}
	else if (simParams.getControlSystem() == "FeedForward") {
		controlSystem = FeedForwardControl::fromOptions(simParams.getControlOptions(), rocketParams);
	}
	else {
		throw std::invalid_argument("Invalid Control System");
	}
	if (simParams.getMotionControlSystem() == "Lookahead") {
		motionControlSystem = LookaheadMotionControlSystem::getFromString(controlSystem, simParams.getMotionControlOptions());
	}
	else {
		throw std::invalid_argument("Invalid Motion Control System");
	}


	return SimSetup(motionControlSystem, rocketParams);
}
std::shared_ptr<MotionControlSystem> SimSetup::getMotionControlSystem()
{
	return motionControlSystem;
}
RocketParams SimSetup::getRocketParams()
{
	return rocketParams;
}
//need to define string based constructors