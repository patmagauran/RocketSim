#pragma once
#include "Control/Trajectory/MotionControlSystem.h"
#include "Model/RocketParams.h"
#include "SimParams.h"
class SimSetup
{
	std::shared_ptr<MotionControlSystem> motionControlSystem;
	RocketParams rocketParams;
public:
	SimSetup(std::shared_ptr<MotionControlSystem> motionControlSystem, RocketParams rocketParams);
	static SimSetup fromCLIFlags(int argc, char* argv[]);
	static SimSetup fromCSVRow(std::vector<std::string> csvRow, int controlSysCol, int motionControlCol, int rocketModelCol);
	static SimSetup fromParameters(
		SimParams simParams
	);
	std::shared_ptr<MotionControlSystem> getMotionControlSystem();
	RocketParams getRocketParams();
};

