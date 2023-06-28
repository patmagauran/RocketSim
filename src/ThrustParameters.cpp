#include "ThrustParameters.h"

ThrustParameters::ThrustParameters(float pitchAngle, float yawAngle, float force)
{
this->pitchAngle = pitchAngle;
	this->yawAngle = yawAngle;
	this->force = force;
}


ChVector<> pitchYawToCartesian(float pitch, float yaw) {
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
