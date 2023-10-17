#pragma once
#include "../LookaheadSystem/LookaheadMotionControlSystem.h"
using namespace chrono;
class SpiralControlSystem :
    public MotionControlSystem
{
protected:
	std::shared_ptr<ControlSystem> controlSystem;
	Course course;
	double lookahead;
	ChVector<> lastGoodPoint;
	double computeAngle(ChVector<> g_location, ChVector<> lookaheadPt);
	double getYawAngle(ChVector<> currentPoint, ChVector<> lookaheadPoint, ChVector<> currentVelocity);
	double getPitchAngle(ChVector<> currentPoint, ChVector<> lookaheadPoint, ChVector<> currentVelocity);
public:
	SpiralControlSystem(std::shared_ptr<ControlSystem> controlSystem, Course course, double lookahead);
	static std::shared_ptr < SpiralControlSystem> getFromString(std::shared_ptr<ControlSystem> controlSystem, std::array<std::string, NUM_MOTION_CONTROL_OPTIONS> options);
	TrajectoryCommand getNextTrajectoryCommand(ChVector<> currentPosition, ChVector<> currentVelocity);
	MotionCommand getNextMotionCommand(ChVector<> g_location, RocketModel rocket, double currentTime);
	std::vector <ChVector<>> getWaypoints();
	void tune(Simulator* sim);
	double distanceFromTrajectory(ChVector<> currentPosition);

};

