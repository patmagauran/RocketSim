#include "NullMotionControlSystem.h"

NullMotionControlSystem::NullMotionControlSystem()
{
}

TrajectoryCommand NullMotionControlSystem::getNextTrajectoryCommand(ChVector<> currentPosition, ChVector<> currentVelocity)
{
	std::cerr << "Using NULL Motion Control System. This shouldnt be happening!" << std::endl;
	return TrajectoryCommand();
}

MotionCommand NullMotionControlSystem::getNextMotionCommand(ChVector<> g_location, RocketModel rocket, double currentTime)
{
	std::cerr << "Using NULL Motion Control System. This shouldnt be happening!" << std::endl;
	return MotionCommand();
}

std::vector<ChVector<>> NullMotionControlSystem::getWaypoints()
{
	std::cerr << "Using NULL Motion Control System. This shouldnt be happening!" << std::endl;
	return std::vector<ChVector<>>();
}

void NullMotionControlSystem::tune(Simulator* sim)
{
	std::cerr << "Using NULL Motion Control System. This shouldnt be happening!" << std::endl;
}

ChVector<> NullMotionControlSystem::getClosestPoint(ChVector<> currentPosition)
{
	std::cerr << "Using NULL Motion Control System. This shouldnt be happening!" << std::endl;

	return ChVector<>();
}

double NullMotionControlSystem::getPercentComplete(ChVector<> closestPoint)
{
	std::cerr << "Using NULL Motion Control System. This shouldnt be happening!" << std::endl;

	return 0.0;
}

