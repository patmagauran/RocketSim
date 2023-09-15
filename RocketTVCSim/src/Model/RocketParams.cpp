#include "RocketParams.h"

RocketParams::RocketParams(double rocket_radius, double lengthAG, double lengthGB, double rocket_mass, double maxThrustAngle, double maxRotationRate, double maxThrust) : rocket_radius{ rocket_radius }, lengthAG{ lengthAG }, lengthGB{ lengthGB }, rocket_mass{ rocket_mass }, maxThrustAngle{ maxThrustAngle }, maxRotationRate{ maxRotationRate }, maxThrust{ maxThrust }
{
}

RocketParams RocketParams::fromOptions(std::array<std::string, NUM_ROCKET_PARAMS> options)
{
    double rocket_radius = std::stod(options[0]);
    double lengthAG = std::stod(options[1]);
    double lengthGB = std::stod(options[2]);
    double rocket_mass = std::stod(options[3]);
    double maxThrustAngle = std::stod(options[4]);
    double maxRotationRate = std::stod(options[5]);
    double maxThrust = std::stod(options[6]);
    return RocketParams(rocket_radius, lengthAG, lengthGB, rocket_mass, maxThrustAngle, maxRotationRate, maxThrust);
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
