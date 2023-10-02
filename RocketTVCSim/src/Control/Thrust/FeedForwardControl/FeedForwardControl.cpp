#include "FeedForwardControl.h"
#include "../../../Util/Utils.h"

FeedForwardControl::FeedForwardControl(RocketParams rocketParams) : rocketParams(rocketParams)
{
}

ControlSystemType FeedForwardControl::getControlSystemType()
{
	return ControlSystemType::FEED_FORWARD;
}

void FeedForwardControl::setParamsThrustAngleFromRate(PIDParams params)
{
}

void FeedForwardControl::setParamsRateFromAngle(PIDParams params)
{
}

PIDParams FeedForwardControl::getParamsThrustAngleFromRate()
{
	return PIDParams(0,0,0);
}

PIDParams FeedForwardControl::getParamsRateFromAngle()
{
	return PIDParams(0,0,0);
}

double FeedForwardControl::getYawRateFromAngleDeviation(double target, double current, double currentTime)
{
	return 0.0;
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

double FeedForwardControl::getYawThrustAngleFromRateDeviation(double target, double current, double currentTime)
{
	return 0.0;
}

double FeedForwardControl::getPitchRateFromAngleDeviation(double target, double current, double currentTime)
{
	return 0.0;
}

double FeedForwardControl::getPitchThrustAngleFromRateDeviation(double target, double current, double currentTime)
{
	return 0.0;
}
