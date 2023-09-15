#pragma once
#include <string>
#include <string>
#include <sstream>
#include <vector>
#include <array>
#include "Util/defines.h"
class SimParams
{
	std::string controlSystem;
	std::array<std::string, NUM_CONTROL_OPTIONS> controlOptions;
	std::string motionControlSystem;
	std::array<std::string, NUM_MOTION_CONTROL_OPTIONS> motionControlOptions;
	std::string rocketModel;

	std::array<std::string, NUM_ROCKET_PARAMS> rocketOptions;
	public:
		SimParams(std::string controlSystem, std::array<std::string, NUM_CONTROL_OPTIONS> controlOptions, std::string motionControlSystem, std::array<std::string, NUM_MOTION_CONTROL_OPTIONS> motionControlOptions, std::string rocketModel, std::array<std::string, NUM_ROCKET_PARAMS> rocketOptions);
		static SimParams fromCLIFlags(int argc, char* argv[]);
		static SimParams fromCSVRow(std::vector<std::string>, int controlSysCol, int motionControlCol, int rocketModelCol);
		std::string getControlSystem();
		std::array<std::string, NUM_CONTROL_OPTIONS> getControlOptions();
		std::string getMotionControlSystem();
		std::array<std::string, NUM_MOTION_CONTROL_OPTIONS> getMotionControlOptions();

		std::string getRocketModel();
		std::array<std::string, NUM_ROCKET_PARAMS> getRocketOptions();

};