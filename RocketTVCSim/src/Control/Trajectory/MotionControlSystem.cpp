#include "MotionControlSystem.h"
#include "TrajectoryCommand.h"

MotionControlSystem::MotionControlSystem(std::shared_ptr<ControlSystem> controlSystem, Course course, double lookahead) : controlSystem(controlSystem), course(course), lookahead(lookahead), lastGoodPoint(0, 0, 0)
{
}
double getPitchAngle(ChVector<> currentPoint, ChVector<> lookaheadPoint, ChVector<> currentVelocity) {
	//Calculate Angle in the x-y plane
	ChVector<> currentLookaheadVector = lookaheadPoint - currentPoint;
	currentLookaheadVector[0] = 0;
	currentVelocity[0] = 0;
	double div = currentLookaheadVector.Length() * currentVelocity.Length();
	if (div == 0)
	{
		return 0;

	}
	double angle = acos(currentLookaheadVector.Dot(currentVelocity) / (div));
	return angle;
}
double getYawAngle(ChVector<> currentPoint, ChVector<> lookaheadPoint, ChVector<> currentVelocity) {
	//Calculate Angle in the x-z plane
	ChVector<> currentLookaheadVector = lookaheadPoint - currentPoint;
	currentLookaheadVector[2] = 0;
	currentVelocity[2] = 0;
	double div = currentLookaheadVector.Length() * currentVelocity.Length();
	if (div == 0)
	{
		return 0;

	}
	double angle = acos(currentLookaheadVector.Dot(currentVelocity) / (div));
	return angle;
}
TrajectoryCommand MotionControlSystem::getNextTrajectoryCommand(ChVector<> currentPosition, ChVector<> currentVelocity) {


	//ChVector<> currentPoint = currentPosition;
	//ChVector<> currentVelocity = currentVelocity;
	ChVector<> lookaheadPoint = this->course.getLookaheadPoint(currentPosition, this->lookahead);
	if (lookaheadPoint == NULL) {
		//whenever lookahead is null, we should continue to aim towards last good point
		lookaheadPoint = this->lastGoodPoint;
	}
	else {
		this->lastGoodPoint = lookaheadPoint;
	}
	double pitchAccel = getPitchAngle(currentPosition, lookaheadPoint, currentVelocity);
	double yawAccel = getYawAngle(currentPosition, lookaheadPoint, currentVelocity);
	if (std::isnan(pitchAccel))
	{
		pitchAccel = 0;
	}
	if (std::isnan(yawAccel))
	{
		yawAccel = 0;
	}
	pitchAccel = -pitchAccel;
	return TrajectoryCommand(pitchAccel, yawAccel, lookaheadPoint);

}

MotionCommand MotionControlSystem::getNextMotionCommand(ChVector<> g_location, RocketModel rocket, double currentTime)
{
	std::shared_ptr<ChBody> rocket_upper = rocket.getRocketUpper();
	ChVector<> currentVelocity = rocket_upper->GetPos_dt();
	if (currentVelocity.Length() < 0.05)
	{
		currentVelocity = (rocket_upper->GetPos() - g_location).GetNormalized();
	}
	//MotionCommand nextCommand = getNextCommand(g_location, currentVelocity);
	TrajectoryCommand nextCommand = getNextTrajectoryCommand(g_location, currentVelocity);
	ChVector<> rotation = rocket_upper->GetRot().Q_to_Euler123();


	double yawAngleO = this->controlSystem->getYawRateFromAngleDeviation(nextCommand.yawAngle, rotation.z(), currentTime);

	double yawThrustAng = this->controlSystem->getYawThrustAngleFromRateDeviation(yawAngleO, rocket_upper->GetWvel_loc().z(), currentTime);

	double pitchAngleO = this->controlSystem->getPitchRateFromAngleDeviation(nextCommand.pitchAngle, rotation.x(), currentTime);

	double pitchThrustAng = this->controlSystem->getPitchThrustAngleFromRateDeviation(pitchAngleO, rocket_upper->GetWvel_loc().x(), currentTime);

	//return MotionCommand(ThrustParameters(0,0,0));
	return MotionCommand(ThrustParameters(pitchThrustAng, yawThrustAng, rocket.getMaxThrust()), nextCommand);
}

std::vector<ChVector<>> MotionControlSystem::getWaypoints()
{
	return this->course.getWaypoints();
}
