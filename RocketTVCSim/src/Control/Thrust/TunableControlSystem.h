#pragma once
#include "ControlSystemTuner.h"
#include "ControlSystem.h"
#include "ITunable.h"
class Simulator;
class TunableControlSystem :public virtual ControlSystem
{
public:
	//TunableControlSystem(ControlSystemTuner tuner, PIDParams rateParams, PIDParams angleParams);
	virtual void tune( Simulator* sim) = 0;
};

