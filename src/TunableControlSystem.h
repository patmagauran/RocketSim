#pragma once
#include "ControlSystemTuner.h"
#include "ControlSystem.h"
class TunableControlSystem : public ControlSystem
{
public:
	TunableControlSystem(ControlSystemTuner tuner, PIDParams rateParams, PIDParams angleParams);
	void tune();
};

