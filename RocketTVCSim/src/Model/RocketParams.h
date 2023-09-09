#pragma once
class RocketParams
{
	double rocket_radius; 
	double lengthAG; 
	double lengthGB; 
	double rocket_mass; 
	double maxThrustAngle; 
	double maxRotationRate; 
	double maxThrust;
	public:
	RocketParams(double rocket_radius, double lengthAG, double lengthGB, double rocket_mass, double maxThrustAngle, double maxRotationRate, double maxThrust);
	RocketParams();
	~RocketParams();
	double getRocketRadius();
	double getLengthAG();
	double getLengthGB();
	double getRocketMass();
	double getMaxThrustAngle();
	double getMaxRotationRate();
	double getMaxThrust();
};

