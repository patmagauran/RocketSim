#include "MotionCommand.h"

MotionCommand::MotionCommand(): MotionCommand(ThrustParameters(0,0,0))
{
}

MotionCommand::MotionCommand(ThrustParameters thrustParameters): thrustParameters(thrustParameters)
{
}

ThrustParameters MotionCommand::getThrustParameters()
{
	return this->thrustParameters;
}
