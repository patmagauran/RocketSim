#pragma once
#include "../Util/defines.h"
#include <array>
#include <string>
class RocketParams
{
	double rocket_radius; 
	double lengthAG; 
	double lengthGB; 
	double rocket_mass; 
	double maxThrustAngle; 
	double maxRotationRate; 
	double maxThrust;
	double dragCoefficient;
	public:
	RocketParams(double rocket_radius, double lengthAG, double lengthGB, double rocket_mass, double maxThrustAngle, double maxRotationRate, double maxThrust, double dragCoefficient = 0);
	static RocketParams fromOptions(std::array<std::string, NUM_ROCKET_PARAMS> options);
	RocketParams();
	~RocketParams();
	double getRocketRadius();
	double getLengthAG();
	double getLengthGB();
	double getRocketMass();
	double getMaxThrustAngle();
	double getMaxRotationRate();
	double getMaxThrust();
	double getDragCoefficient();
};

