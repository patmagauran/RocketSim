#include "RocketModel.h"



/*
Make 2 cylinders joined by a fixed link

*/


using namespace chrono;
using namespace chrono::irrlicht;


std::shared_ptr < ChBody > RocketModel::makeCylinder(double radius, double length, ChColor color, std::shared_ptr< ChMaterialSurface > material, double mass) {
	auto cylinder = std::make_shared<ChBody>();

	double density = mass / (CH_C_PI * pow(radius, 2) * length);
	cylinder->SetDensity(density);
	cylinder->SetMass(mass);
	cylinder->SetInertiaXX(ChVector<double>((1.0 / 12.0) * mass * (3 * pow(radius, 2) + pow(length, 2)),
		0.5 * mass * pow(radius, 2),
		(1.0 / 12.0) * mass * (3 * pow(radius, 2) + pow(length, 2))));
	cylinder->GetCollisionModel()->ClearModel();
	cylinder->GetCollisionModel()->AddCylinder(material, radius, radius, length / 2);
	cylinder->GetCollisionModel()->BuildModel();
	cylinder->SetCollide(true);

	auto vshape = std::make_shared<ChCylinderShape>();
	vshape->GetGeometry().r = radius;
	vshape->GetGeometry().h = length;
	vshape->SetColor(color);
	
	auto vmodel = std::make_shared<ChVisualModel>();
	vmodel->AddShape(vshape, ChFrame<>(ChVector<>(0,0,0), ChQuaternion<>(0, 0, 0.7071068, 0.7071068)));
	cylinder->AddVisualModel(vmodel);

	return cylinder;
}

RocketModel::RocketModel(double rocket_radius, double lengthAG, double lengthGB, double rocket_mass)
{

	auto material = std::make_shared<ChMaterialSurfaceNSC>();
	double comp_mass = rocket_mass / 2;
	this->rocket_lower = makeCylinder(rocket_radius, lengthAG, ChColor(1,1,1), material, comp_mass);
	this->rocket_upper = makeCylinder(rocket_radius, lengthGB, ChColor(1,1,0.2), material, comp_mass);
	this->rocket_upper->SetPos(ChVector<>(0, lengthAG/2, 0));
	this->rocket_upper->SetPos_dt(ChVector<>(0, 0, 0));
	this->rocket_lower->SetPos(ChVector<>(0, -lengthGB/2, 0));
	this->rocket_lower->SetPos_dt(ChVector<>(0, 0, 0));
	this->fixed_link = std::make_shared<ChLinkMateFix>();
	this->fixed_link->Initialize(this->rocket_lower, this->rocket_upper, ChFrame<>(ChCoordsys<>(ChVector<>(0, 0, 0))));
	this->thrust_point = ChVector<>(0, -lengthGB / 2, 0);

	
}

RocketModel::~RocketModel()
{
}


void RocketModel::addRocketModelToSystem(chrono::ChSystem& system) {
	system.AddBody(this->rocket_lower);
	system.AddBody(this->rocket_upper);
	system.AddLink(this->fixed_link);

}

void RocketModel::accumulateForces(ChVector<> thrust_force)
{
	this->forces.clear();
	this->rocket_lower->Empty_forces_accumulators();
this->rocket_upper->Empty_forces_accumulators();

this->rocket_lower->Accumulate_force(thrust_force, this->thrust_point, true);
this->forces.push_back(ForceApplication(rocket_lower->Dir_Body2World(thrust_force), rocket_lower->Point_Body2World(this->thrust_point)));
}

std::list<ForceApplication> RocketModel::getDisplayedForces()
{
	return this->forces;
}

ChVector<> RocketModel::getGLocation()
{
	return this->fixed_link->GetLinkAbsoluteCoords().pos;
}

std::shared_ptr<ChBody> RocketModel::getRocketUpper()
{
	return this->rocket_upper;
}

