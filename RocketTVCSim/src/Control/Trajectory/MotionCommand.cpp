#include "MotionCommand.h"

MotionCommand::MotionCommand(): MotionCommand(ThrustParameters(0,0,0), TrajectoryCommand())
{
}

MotionCommand::MotionCommand(ThrustParameters thrustParameters, TrajectoryCommand trajectoryCommand): thrustParameters(thrustParameters), trajectoryCommand(trajectoryCommand)
{
}

ThrustParameters MotionCommand::getThrustParameters()
{
	return this->thrustParameters;
}

TrajectoryCommand MotionCommand::getTrajectoryCommand()
{
	return trajectoryCommand;
}
