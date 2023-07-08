#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <chrono/core/ChVector.h>
using namespace chrono;
class Course
{
	std::vector <ChVector<>> waypoints;
	int waypointIndex = 0;
public:
	Course(std::string fileName);
	std::vector <ChVector<>> getWaypoints();
	ChVector<> getLookaheadPoint(ChVector<> currentPosition, double lookahead);
};

