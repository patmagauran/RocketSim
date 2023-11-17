#pragma once
#include "ControlSystem.h"
class Simulator;
class TunableControlSystem :public virtual ControlSystem
{
public:
	//TunableControlSystem(ControlSystemTuner tuner, PIDParams rateParams, PIDParams angleParams);
	virtual void tune( Simulator* sim) = 0;
};

