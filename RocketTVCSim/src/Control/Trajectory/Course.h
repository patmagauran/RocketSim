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
	ChVector<> getClosestPoint(ChVector<> currentPosition);
	ChVector<> getClosestPointOnSegment(ChVector<> currentPosition, ChVector<> segmentStart, ChVector<> segmentEnd);
	double getPercentComplete(ChVector<> closestPoint);
	bool isPointOnSegment(ChVector<> currentPosition, ChVector<> segmentStart, ChVector<> segmentEnd);
};

