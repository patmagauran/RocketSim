#include "TunableControlSystem.h"

TunableControlSystem::TunableControlSystem(ControlSystemTuner tuner, PIDParams rateParams, PIDParams angleParams) : ControlSystem{ rateParams, angleParams }
{
	
}

void TunableControlSystem::tune()
{
}
