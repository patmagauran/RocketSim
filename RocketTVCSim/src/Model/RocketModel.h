#pragma once

#include <physics/ChSystem.h>
#include <irrlicht.h>
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkMate.h"
#include "../Util/ForceApplication.h"
#include "RocketParams.h"
using namespace chrono;
using namespace chrono::irrlicht;
class RocketModel
{	
	protected:
	std::shared_ptr < ChBody > makeCylinder(double radius, double length, ChColor color, std::shared_ptr< ChMaterialSurface > material, double mass);
	std::shared_ptr < ChBody > rocket_upper;
	std::shared_ptr < ChBody > rocket_lower;
	std::shared_ptr <ChLinkMateFix> fixed_link;
	ChVector<> thrust_point;
	std::list<ForceApplication> forces;
	double maxThrust = 5;
	double maxThrustAngle=1000, maxRotationRate=1000;
public:
	RocketModel(double rocket_radius, double lengthAG, double lengthGB, double rocket_mass, double maxThrustAngle, double maxRotationRate, double maxThrust);
	RocketModel(RocketParams params);
	~RocketModel();
	void addRocketModelToSystem(chrono::ChSystem &system, chrono::ChVisualSystem &vis);
	void accumulateForces(ChVector<> thrust_force);
	void accumulateDrag(ChVector<> wind);
	std::list<ForceApplication> getDisplayedForces();
	ChVector<> getGLocation();
	std::shared_ptr<ChBody> getRocketUpper();
	double getMaxThrust();
	double getMaxThrustAngle();
	double getMaxRotationRate();
	void logRocketData();
	void reset(RocketParams params);

};

