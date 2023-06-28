#pragma once

#include <chrono/core/ChVector.h>
class ForceApplication
{
public:
	chrono::ChVector<> force;
	chrono::ChVector<> point;
    ForceApplication(chrono::ChVector<> force, chrono::ChVector<> point);
};

