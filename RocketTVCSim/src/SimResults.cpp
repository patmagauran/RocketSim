#include "SimResults.h"
#include <iostream>

SimResults::SimResults() : name(""), worldRunTime(0), simulationRunTime(0), resultType(SimResultType::RESULT_TYPE_NONE), maxDistanceFromCourse(0), distanceAlongCourse(0), score(0)
{}

SimResults::SimResults(std::string name, double worldRunTime, double simulationRunTime, SimResultType resultType, double maxDistanceFromCourse, double distanceAlongCourse, double score) : name(name), worldRunTime(worldRunTime), simulationRunTime(simulationRunTime), resultType(resultType), maxDistanceFromCourse(maxDistanceFromCourse), distanceAlongCourse(distanceAlongCourse), score(score)
{
}

void SimResults::printResults()
{
	std::cout << "Name: " << name << std::endl;
	std::cout << "World Run Time: " << worldRunTime << std::endl;
	std::cout << "Simulation Run Time: " << simulationRunTime << std::endl;
	std::cout << "Result Type: " << resultType << std::endl;
	std::cout << "Max Distance From Course: " << maxDistanceFromCourse << std::endl;
	std::cout << "Distance Along Course: " << distanceAlongCourse << std::endl;
	std::cout << "Score: " << score << std::endl;
}

std::vector<std::string> SimResults::getResultsArray()
{
	std::vector<std::string> results;
	results.push_back(name);
	results.push_back(std::to_string(worldRunTime));
	results.push_back(std::to_string(simulationRunTime));
	results.push_back(std::to_string(resultType));
	results.push_back(std::to_string(maxDistanceFromCourse));
	results.push_back(std::to_string(distanceAlongCourse));
	results.push_back(std::to_string(score));
	return results;
}
