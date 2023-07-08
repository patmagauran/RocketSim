#pragma once
#include "../Thrust/ThrustParameters.h"
#include "TrajectoryCommand.h"
class MotionCommand
{
	ThrustParameters thrustParameters;
	TrajectoryCommand trajectoryCommand;
public:
MotionCommand();
	MotionCommand(ThrustParameters thrustParameters, TrajectoryCommand trajectoryCommand);
	ThrustParameters getThrustParameters();
	TrajectoryCommand getTrajectoryCommand();
};

