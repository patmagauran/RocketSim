#pragma once
#include "../TunableControlSystem.h"
#include "../ControlSystemTuner.h"
#include "PIDControlSystem.h"
#include "../ITunable.h"
#include "../../../Util/DataLog/DataLog.h"
#include "../ThrustParameters.h"
#include "../PID/PIDAutoTuner.h"
#include "../../../Util/Utils.h"
#include "../../../Simulator.h"
class TunablePIDControlSystem :
     public PIDControlSystem, public virtual TunableControlSystem
{
public:
    using PIDControlSystem::PIDControlSystem;
    using PIDControlSystem::setParamsRate;
    using PIDControlSystem::setParamsAngle;
    using PIDControlSystem::getParamsRate;
    using PIDControlSystem::getParamsAngle;
    using PIDControlSystem::getYawAngle;
    using PIDControlSystem::getYawRate;
    using PIDControlSystem::getPitchAngle;
    using PIDControlSystem::getPitchRate;

    TunablePIDControlSystem(ControlSystemTuner tuner, PIDParams rateParams, PIDParams angleParams);
    virtual void tune(double maxDeflection, double maxRotationAngle, Simulator* sim) override;
};

