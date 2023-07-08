#pragma once

#include "chrono/core/ChVector.h"

using namespace chrono;
class ThrustParameters
{
public:
	double pitchAngle, yawAngle, force;
ThrustParameters(double pitchAngle, double yawAngle, double force);
ChVector<> convertToForceVector();
};

