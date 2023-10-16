#include "RocketModelWithDrag.h"
#define DRAG_SPEED_CUTTOFF 0.0001
RocketModelWithDrag::RocketModelWithDrag(RocketParams params) : upperHeight(params.getLengthAG()), lowerHeight(params.getLengthGB()), upperDragPt(ChVector<>(0, params.getLengthAG() / 2, 0)), lowerDragPt(ChVector<>(0, -params.getLengthGB() / 2, 0)), radius(params.getRocketRadius()), dragCoefficient(params.getDragCoefficient()), RocketModel(params)
{
}

RocketModelWithDrag::RocketModelWithDrag(double rocket_radius, double lengthAG, double lengthGB, double rocket_mass, double maxThrustAngle, double maxRotationRate, double maxThrust, double dragCoefficient) : upperHeight(lengthAG), lowerHeight(lengthGB), upperDragPt(ChVector<>(0, lengthAG / 2, 0)), lowerDragPt(ChVector<>(0, -lengthGB / 2, 0)), radius(rocket_radius), dragCoefficient(dragCoefficient), RocketModel(rocket_radius, lengthAG, lengthGB, rocket_mass, maxThrustAngle, maxRotationRate, maxThrust)
{
}

RocketModelWithDrag::~RocketModelWithDrag()
{
}

void RocketModelWithDrag::accumulateDrag(ChVector<> wind)
{
	if (this->dragCoefficient <= 0) {
		return;
	}
	//Drag due to wind is a force applied at the center of the front face, and the center of each half of the rocket. 
	//The effective wind is the addition product of the wind vector and the velocity of the rocket.
	//Its value is then the magnitude of the wind vector times the magnitude of the velocity vector times the drag coefficient of the rocket * the cross sectional area of the rocket.
	//The final force value is then dotted with the normal of the face to get the amount applied.
	//We will assume the surface is designed with negligible shear drag, so drag will always be normal to the face to whatever degree it is applied.
	//The force is then applied to the center of the face.

	//Add Global speed of rocket to wind
	ChVector<> effective_wind_l = rocket_upper->Dir_World2Body(wind) - this->rocket_lower->GetPos_dt();
	ChVector<> effective_wind_u = rocket_upper->Dir_World2Body(wind) - this->rocket_upper->GetPos_dt();
	if (effective_wind_l.Length() <= DRAG_SPEED_CUTTOFF && effective_wind_u.Length() <= DRAG_SPEED_CUTTOFF) {
		return;
	}
	//Rocket orientation
	ChQuaternion<> rocket_orientation = this->rocket_lower->GetRot();

	//double drag_coef = 0.5;

	//double front_Component = effective_wind.Dot(rocket_orientation.GetZaxis());
	ChVector<> side_wind_l = effective_wind_l * ChVector<>(1, 0, 1);
	ChVector<> side_wind_u = effective_wind_u * ChVector<>(1, 0, 1);
	if (side_wind_l <= DRAG_SPEED_CUTTOFF && side_wind_u <= DRAG_SPEED_CUTTOFF) {
		return;
	}
	ChVector<> side_component_l = 0.5 * ChVector<>(this->dragCoefficient,0, this->dragCoefficient) * effective_wind_l.Length2() * (effective_wind_l.GetNormalized());
	ChVector<> side_component_u = 0.5 * ChVector<>(this->dragCoefficient,0, this->dragCoefficient) * effective_wind_u.Length2() * (effective_wind_u.GetNormalized());
	//Front
	//double front_drag_force = 0;
	//Upper
	// 0.5 * 2*r*pi*h
	double upperCrossSectionalArea = 3.14159 * this->upperHeight * this->radius;
	//Lower
	double lowerCrossSectionalArea = 3.14159 * this->lowerHeight * this->radius;

	//We should also consider rotational speed of the rocket, as this will also cause drag.



	this->rocket_upper->Accumulate_force(side_component_u * upperCrossSectionalArea, this->upperDragPt, true);
	this->rocket_lower->Accumulate_force(side_component_l *lowerCrossSectionalArea, this->lowerDragPt, true);

	this->forces.push_back(ForceApplication(rocket_upper->Dir_Body2World(side_component_u * upperCrossSectionalArea), rocket_upper->Point_Body2World(this->upperDragPt)));
	this->forces.push_back(ForceApplication(rocket_lower->Dir_Body2World(side_component_l * lowerCrossSectionalArea), rocket_lower->Point_Body2World(this->lowerDragPt)));
}

void RocketModelWithDrag::reset(RocketParams params)
{
	this -> upperHeight = params.getLengthAG();
	this -> lowerHeight = params.getLengthGB();
	this -> upperDragPt = ChVector<>(0, params.getLengthAG() / 4, 0);
	this -> lowerDragPt = ChVector<>(0, -params.getLengthGB() / 4, 0);
	this -> radius = params.getRocketRadius();
	this -> dragCoefficient = params.getDragCoefficient();
	RocketModel::reset(params);
}
