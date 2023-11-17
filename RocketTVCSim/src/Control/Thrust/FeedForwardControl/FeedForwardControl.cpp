#include "FeedForwardControl.h"
#include "../../../Util/Utils.h"
#include "../../Trajectory/TrajectoryCommand.h"
#include "../ThrustParameters.h"
#include "../../../Model/ControlState.h"
FeedForwardControl::FeedForwardControl(RocketParams rocketParams) : rocketParams(rocketParams)
{
}

ControlSystemType FeedForwardControl::getControlSystemType()
{
	return ControlSystemType::FEED_FORWARD;
}

	

double FeedForwardControl::ComputeAngle(double velocity, double theta, double ang_velocity) {
	
	//x = sin ^ (-1)((K(mS(S + sqrt(S ^ 2 + (2T) / (m)))(theta - 1) + Ttheta)) / (LmT))
	//arcsin((L ^ 2⋅m⋅(theta - t⋅w)) / (6⋅B⋅T⋅t ^ 2))
	double m = rocketParams.getRocketMass();
	double T = rocketParams.getMaxThrust();
	double B = rocketParams.getLengthGB();
	double L = rocketParams.getLengthAG() +B;
	double t = lookaheadDistance / velocity;
	double x = asin((((L * L) * m) * (theta - (t * ang_velocity))) / (((6 * B) * T) * (t * t)));
	return clamp(x, rocketParams.getMaxThrustAngle());

}

std::shared_ptr<FeedForwardControl> FeedForwardControl::fromOptions(std::array<std::string, NUM_CONTROL_OPTIONS> options, RocketParams rocketParams)
{
	return std::make_shared<FeedForwardControl>(rocketParams);
}

ThrustParameters FeedForwardControl::computeThrustParameters(ControlState currentState, TrajectoryCommand command, double currentTime)
{
	double yawThrustAng = ComputeAngle(currentState.velocity, command.yawAngle - currentState.yawAngle, currentState.wvely);
	double pitchThrustAng = ComputeAngle(currentState.velocity, command.pitchAngle - currentState.pitchAngle, currentState.wvelx);
	return ThrustParameters(yawThrustAng, pitchThrustAng, currentState.maxThrust);
}
