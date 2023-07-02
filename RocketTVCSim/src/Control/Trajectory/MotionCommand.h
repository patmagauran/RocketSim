#pragma once
#include "../Thrust/ThrustParameters.h"
class MotionCommand
{
	ThrustParameters thrustParameters;
public:
MotionCommand();
	MotionCommand(ThrustParameters thrustParameters);
	ThrustParameters getThrustParameters();
};

