#include "RocketParams.h"

RocketParams::RocketParams(double rocket_radius, double lengthAG, double lengthGB, double rocket_mass, double maxThrustAngle, double maxRotationRate, double maxThrust) : rocket_radius{ rocket_radius }, lengthAG{ lengthAG }, lengthGB{ lengthGB }, rocket_mass{ rocket_mass }, maxThrustAngle{ maxThrustAngle }, maxRotationRate{ maxRotationRate }, maxThrust{ maxThrust }
{
}

RocketParams::RocketParams()
{
}

RocketParams::~RocketParams()
{
}

double RocketParams::getRocketRadius()
{
    return rocket_radius;
}

double RocketParams::getLengthAG()
{
    return lengthAG;
}

double RocketParams::getLengthGB()
{
    return lengthGB;
}

double RocketParams::getRocketMass()
{
    return rocket_mass;
}

double RocketParams::getMaxThrustAngle()
{
    return maxThrustAngle;
}

double RocketParams::getMaxRotationRate()
{
    return maxRotationRate;
}

double RocketParams::getMaxThrust()
{
    return maxThrust;
}
