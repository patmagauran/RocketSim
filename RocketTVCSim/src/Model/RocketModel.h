#pragma once

#include <physics/ChSystem.h>
#include <irrlicht.h>
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkMate.h"
#include "../Util/ForceApplication.h"

using namespace chrono;
using namespace chrono::irrlicht;
class RocketModel
{	
	std::shared_ptr < ChBody > makeCylinder(float radius, float length, ChColor color, std::shared_ptr< ChMaterialSurface > material, float mass);
	std::shared_ptr < ChBody > rocket_upper;
	std::shared_ptr < ChBody > rocket_lower;
	std::shared_ptr <ChLinkMateFix> fixed_link;
	ChVector<> thrust_point;
	std::list<ForceApplication> forces;
	public:
	RocketModel(float rocket_radius, float lengthAG, float lengthGB, float rocket_mass);
	~RocketModel();
	void addRocketModelToSystem(chrono::ChSystem &system);
	void accumulateForces(ChVector<> thrust_force);
	std::list<ForceApplication> getDisplayedForces();
	ChVector<> getGLocation();
	std::shared_ptr<ChBody> getRocketUpper();
};
