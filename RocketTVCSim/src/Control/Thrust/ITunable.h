#pragma once
class ITunable
{
public:
	//TunableControlSystem(ControlSystemTuner tuner, PIDParams rateParams, PIDParams angleParams);
	virtual void tune() = 0;
};