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
public:
	Course(std::string fileName);
	std::vector <ChVector<>> getWaypoints();
};

