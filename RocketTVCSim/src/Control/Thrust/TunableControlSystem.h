#pragma once
#include "ControlSystemTuner.h"
#include "ControlSystem.h"
#include "ITunable.h"
#include "../../Simulator.h"
class TunableControlSystem :public virtual ControlSystem
{
public:
	//TunableControlSystem(ControlSystemTuner tuner, PIDParams rateParams, PIDParams angleParams);
	virtual void tune(double maxDeflection, double maxRotationAngle, Simulator* sim) = 0;
};

