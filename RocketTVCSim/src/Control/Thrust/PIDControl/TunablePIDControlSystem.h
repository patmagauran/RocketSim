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
    std::string rateFromAngleRule;
    std::string thrustFromRateRule;
public:
    using PIDControlSystem::PIDControlSystem;
    using PIDControlSystem::setParamsThrustAngleFromRate;
    using PIDControlSystem::setParamsRateFromAngle;
    using PIDControlSystem::getParamsThrustAngleFromRate;
    using PIDControlSystem::getParamsRateFromAngle;
    using PIDControlSystem::getYawRateFromAngleDeviation;
    using PIDControlSystem::getYawThrustAngleFromRateDeviation;
    using PIDControlSystem::getPitchRateFromAngleDeviation;
    using PIDControlSystem::getPitchThrustAngleFromRateDeviation;

    TunablePIDControlSystem(PIDParams paramsThrustAngleFromRate, PIDParams paramsRateFromAngle, std::string rateFromAngleRule = "ziegler-nichols", std::string thrustFromRateRule = "ziegler-nichols");
    virtual void tune(Simulator* sim) override;
};

