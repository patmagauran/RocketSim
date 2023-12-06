#pragma once
#include <string>
#include <vector>
#include <core/ChVector.h>

enum SimResultType
{
	RESULT_TYPE_NONE,
	RESULT_TYPE_SUCCESS,
	RESULT_TYPE_FAIL_DIST,
	RESULT_TYPE_FAIL_ROT,
	RESULT_TYPE_ERROR
};

class SimResults
{

public:
	std::string name;
	double worldRunTime;
	double simulationRunTime;
	SimResultType resultType;
	double maxDistanceFromCourse;
	double distanceAlongCourse;
	double score;
	chrono::ChVector<> closestPosition;


SimResults();
	SimResults(std::string name, double worldRunTime, double simulationRunTime, SimResultType resultType, double maxDistanceFromCourse, double distanceAlongCourse, double score);

	void printResults();

	std::vector<std::string> getResultsArray();
};

