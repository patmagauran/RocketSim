#pragma once
#include "../MotionControlSystem.h"
class NullMotionControlSystem :
    public MotionControlSystem
{
    public:
	NullMotionControlSystem();
	virtual TrajectoryCommand getNextTrajectoryCommand(ChVector<> currentPosition, ChVector<> currentVelocity) override;
	virtual MotionCommand getNextMotionCommand(ChVector<> g_location, RocketModel rocket, double currentTime) override;
	virtual std::vector <ChVector<>> getWaypoints() override;
	virtual void tune(Simulator* sim) override;
	virtual ChVector<> getClosestPoint(ChVector<> currentPosition) override;
	virtual double getPercentComplete(ChVector<> closestPoint) override;

};

