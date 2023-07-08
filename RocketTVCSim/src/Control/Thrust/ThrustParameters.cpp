#include "ThrustParameters.h"

ThrustParameters::ThrustParameters(double pitchAngle, double yawAngle, double force)
{
this->pitchAngle = pitchAngle;
	this->yawAngle = yawAngle;
	this->force = force;
}


ChVector<> pitchYawToCartesian(double pitch, double yaw) {
	return ChVector<>(
		sin(yaw),
		pow(cos(yaw),2) - pow(sin(pitch),2),
		sin(pitch)
	).GetNormalized();
}

ChVector<> ThrustParameters::convertToForceVector()
{
return pitchYawToCartesian(-pitchAngle, yawAngle) * force;
}
