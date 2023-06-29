#pragma once
#include "ThrustParameters.h"
class MotionCommand
{
	ThrustParameters thrustParameters;
public:
MotionCommand();
	MotionCommand(ThrustParameters thrustParameters);
	ThrustParameters getThrustParameters();
};

