#pragma once

#include "chrono/core/ChVector.h"

using namespace chrono;
class ThrustParameters
{
public:
	float pitchAngle, yawAngle, force;
ThrustParameters(float pitchAngle, float yawAngle, float force);
ChVector<> convertToForceVector();
};

