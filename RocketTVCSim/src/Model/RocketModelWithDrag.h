#pragma once
#include "RocketModel.h"
class RocketModelWithDrag: public RocketModel
{
	double dragCoefficient = 0.5;
	ChVector<> upperDragPt = ChVector<>(0, 0, 0);
	ChVector<> lowerDragPt = ChVector<>(0, 0, 0);
	double upperHeight;
	double lowerHeight;
	double radius;
public:
	RocketModelWithDrag(double rocket_radius, double lengthAG, double lengthGB, double rocket_mass, double maxThrustAngle, double maxRotationRate, double maxThrust, double dragCoefficient);
	RocketModelWithDrag(RocketParams params);
	~RocketModelWithDrag();
	void accumulateDrag(ChVector<> wind);

	void reset(RocketParams params);

};

