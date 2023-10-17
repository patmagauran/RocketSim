#include "SpiralControlSystem.h"
#include "../TrajectoryCommand.h"
#include "../../Thrust/TunableControlSystem.h"
#include "../../Thrust/FeedForwardControl/FeedForwardControl.h"

SpiralControlSystem::SpiralControlSystem(std::shared_ptr<ControlSystem> controlSystem, Course course, double lookahead) : controlSystem(controlSystem), course(course), lookahead(lookahead), lastGoodPoint(0, 0, 0)
{
}

std::shared_ptr < SpiralControlSystem> SpiralControlSystem::getFromString(std::shared_ptr<ControlSystem> controlSystem, std::array<std::string, NUM_MOTION_CONTROL_OPTIONS> options)
{
	std::string courseFile = options[0];
	double lookahead = std::stod(options[1]);
	Course course = Course(courseFile);
	return std::make_shared<SpiralControlSystem>(controlSystem, course, lookahead);
}



double SpiralControlSystem::getPitchAngle(ChVector<> currentPoint, ChVector<> lookaheadPoint, ChVector<> currentVelocity) {
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
double SpiralControlSystem::getYawAngle(ChVector<> currentPoint, ChVector<> lookaheadPoint, ChVector<> currentVelocity) {
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
TrajectoryCommand SpiralControlSystem::getNextTrajectoryCommand(ChVector<> currentPosition, ChVector<> currentVelocity) {


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

double SpiralControlSystem::computeAngle(ChVector<> g_location, ChVector<> lookaheadPt) {
	//Ideally we could compute the angle of thrust to spiral from the current position to the lookahead point
	//Need to work more on math for now
	return 0;
}

MotionCommand SpiralControlSystem::getNextMotionCommand(ChVector<> g_location, RocketModel rocket, double currentTime)
{
	std::shared_ptr<ChBody> rocket_upper = rocket.getRocketUpper();
	ChVector<> currentVelocity = rocket_upper->GetPos_dt();
	if (currentVelocity.Length() < 0.05)
	{
		currentVelocity = (rocket_upper->GetPos() - g_location).GetNormalized();
	}
	//MotionCommand nextCommand = getNextCommand(g_location, currentVelocity);
	ChVector<> lookaheadPoint = this->course.getLookaheadPoint(g_location, this->lookahead);
	ChVector<> rotation = rocket_upper->GetRot().Q_to_Euler123();
	double yawThrustAng, pitchThrustAng;

	
	
	

	//return MotionCommand(ThrustParameters(0,0,0));
	return MotionCommand(ThrustParameters(pitchThrustAng, yawThrustAng, rocket.getMaxThrust()), TrajectoryCommand());
}

std::vector<ChVector<>> SpiralControlSystem::getWaypoints()
{
	return this->course.getWaypoints();
}

void SpiralControlSystem::tune(Simulator* sim)
{
	//Note, Dynamic casting not the ideal way, but it should work and be safe
	TunableControlSystem* ptr = dynamic_cast<TunableControlSystem*>(controlSystem.get());
	if (ptr) {
		ptr->tune(sim);
	}
	else {
		std::cout << "We can't tune using this control system. Skipping tuning." << std::endl;
	}
}

double SpiralControlSystem::distanceFromTrajectory(ChVector<> currentPosition)
{
	//we want the close location along the course to the current position

	//for each segment on course, find closest point to current position
	//Then select the closest of those points
	
	return course.distanceFromTrajectory(currentPosition);
	
}
