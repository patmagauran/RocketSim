#pragma once
#include "../../Thrust/ControlSystem.h"
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChBody.h"
#include "../MotionCommand.h"
#include "../TrajectoryCommand.h"
#include "../Course.h"
#include "../../../Model/RocketModel.h"
#include "../MotionControlSystem.h"
#include "../../../Util/defines.h"
using namespace chrono;
class LookaheadMotionControlSystem :
    public MotionControlSystem
{
protected:
	std::shared_ptr<ControlSystem> controlSystem;
	Course course;
	double lookahead;
	ChVector<> lastGoodPoint;
	double getYawAngle(ChVector<> currentPoint, ChVector<> lookaheadPoint, ChVector<> currentVelocity);
	double getPitchAngle(ChVector<> currentPoint, ChVector<> lookaheadPoint, ChVector<> currentVelocity);
public:
	LookaheadMotionControlSystem(std::shared_ptr<ControlSystem> controlSystem, Course course, double lookahead);
	static std::shared_ptr < LookaheadMotionControlSystem> getFromString(std::shared_ptr<ControlSystem> controlSystem, std::array<std::string, NUM_MOTION_CONTROL_OPTIONS> options);
	TrajectoryCommand getNextTrajectoryCommand(ChVector<> currentPosition, ChVector<> currentVelocity);
	Eigen::Matrix<double, 1, -1> computeDesiredState(ChVector<> currentPosition, ChVector<> currentVelocity, TrajectoryCommand trajectoryCommand, ChVector<> rotation, double currentTime);
	MotionCommand getNextMotionCommand(ChVector<> g_location, RocketModel rocket, double currentTime);
	std::vector <ChVector<>> getWaypoints();
	void tune(Simulator* sim);
	ChVector<> getClosestPoint(ChVector<> currentPosition);
	double getPercentComplete(ChVector<> closestPoint);

};

