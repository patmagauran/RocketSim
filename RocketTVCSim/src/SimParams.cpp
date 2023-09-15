#include "SimParams.h"

using namespace std;
SimParams::SimParams(std::string controlSystem, array<string, NUM_CONTROL_OPTIONS> controlOptions, std::string motionControlSystem, array<string, NUM_MOTION_CONTROL_OPTIONS> motionControlOptions, string rocketModel, array<string, NUM_ROCKET_PARAMS> rocketOptions) : controlSystem(controlSystem), controlOptions(controlOptions), motionControlSystem(motionControlSystem), motionControlOptions(motionControlOptions), rocketModel(rocketModel), rocketOptions(rocketOptions)
{
}
enum SimParamState {
	CONTROL_SYSTEM,
	CONTROL_OPTIONS,
	MOTION_CONTROL_SYSTEM,
	MOTION_CONTROL_OPTIONS,
    ROCKET_MODEL,
    ROCKET_OPTIONS,
    UNKNOWN
};
SimParams SimParams::fromCLIFlags(int argc, char* argv[])
{
    //General format of CLI Flags will be:
    //-c dictates the control system
    //-m dictates the motion control system
    //-o dictates the options for the control system
    //-p dictates the options for the motion control system
    // -r dictates the rocket model
    // -s dictates the options for the rocket model
    //Example: -c PID -o 1,0.1,0.1 -m lookahead -p "pathtocourse",25
    
    //Split cliFlags by spaces
    //Loop through, comparing each flag to the above format
    //If we encounter an option flag, append the next argument to the appropriate option array until we reach the end or another flag
    //If we encounter a system flag, set the appropriate system variable to the next argument and then continue
    std::string controlSystem;
    array<string, NUM_CONTROL_OPTIONS> controlOptions;
    std::string motionControlSystem;
    array<string, NUM_MOTION_CONTROL_OPTIONS> motionControlOptions;
    string parsed;
   // stringstream input_stringstream(cliFlags);
    SimParamState state = UNKNOWN;
    string rocketModel; 
    array<string, NUM_ROCKET_PARAMS> rocketOptions;
    int optionIndex = 0;
    int motionOptionIndex = 0;
  //  if (getline(input_stringstream, parsed, ' '))
    for (int i = 0; i < argc; i++) 
    {
        parsed = argv[i];
        if (parsed == "-c") {
            state = CONTROL_SYSTEM;
        }
        else if (parsed == "-m") {
            state = MOTION_CONTROL_SYSTEM;
        }
        else if (parsed == "-o") {
            state = CONTROL_OPTIONS;
        }
        else if (parsed == "-p") {
            state = MOTION_CONTROL_OPTIONS;
        }
        else if (parsed == "-r") {
			state = ROCKET_MODEL;
        }
        else if (parsed == "-s") {
			state = ROCKET_OPTIONS;
		}
        else {
            switch (state) {
                case CONTROL_SYSTEM:
					controlSystem = parsed;
					break;
                case CONTROL_OPTIONS:
                    //Add to control options
                    controlOptions[optionIndex] = parsed;
                    optionIndex++;
                    break;
                case MOTION_CONTROL_SYSTEM:
                    //Set motion control system
                    motionControlSystem = parsed;
                    break;
                case MOTION_CONTROL_OPTIONS:
                    //Add to motion control options
                    motionControlOptions[motionOptionIndex] = parsed;
                    motionOptionIndex++;
                    break;
                case ROCKET_MODEL:
                    //Set rocket model
					rocketModel = parsed;
					break;
                case ROCKET_OPTIONS:
					//Add to rocket options
					rocketOptions[optionIndex] = parsed;
					optionIndex++;
					break;
                case UNKNOWN:
                    //Do nothing
                    break;
            }
        }
    }
    return SimParams(controlSystem, controlOptions, motionControlSystem, motionControlOptions, rocketModel, rocketOptions);
}

SimParams SimParams::fromCSVRow(std::vector<std::string> csvRow, int controlSysCol, int motionControlCol, int rocketModelCol)
{
    std::string controlSystem;
    array<string, NUM_CONTROL_OPTIONS> controlOptions;
    std::string motionControlSystem;
    array<string, NUM_MOTION_CONTROL_OPTIONS> motionControlOptions;
    string rocketModel;
    array<string, NUM_ROCKET_PARAMS> rocketOptions;

    controlSystem = csvRow.at(controlSysCol);

    //Copy elements from csvRow[controlSysCol+1] to csvRow[controlSysCol+NUM_CONTROL_OPTIONS] into controlOptions
    for (int i = 0; (i < NUM_CONTROL_OPTIONS) && (controlSysCol + i + 1 < motionControlCol); i++) {
		controlOptions[i] = csvRow.at(controlSysCol + i + 1);
	}
    motionControlSystem = csvRow.at(motionControlCol);
    //Copy elements from csvRow[motionControlCol+1] to csvRow[motionControlCol+NUM_MOTION_CONTROL_OPTIONS] into motionControlOptions
    for (int i = 0; (i < NUM_MOTION_CONTROL_OPTIONS) && (motionControlCol + i + 1 < rocketModelCol); i++) {
        motionControlOptions[i] = csvRow.at(motionControlCol + i + 1);
    }
    rocketModel = csvRow.at(rocketModelCol);
    //Copy elements from csvRow[rocketModelCol+1] to csvRow[rocketModelCol+NUM_ROCKET_PARAMS] into rocketOptions
    for (int i = 0; (i < NUM_ROCKET_PARAMS) && (rocketModelCol + i + 1 < csvRow.size()); i++) {
		rocketOptions[i] = csvRow.at(rocketModelCol + i + 1);
	}
    return SimParams(controlSystem, controlOptions, motionControlSystem, motionControlOptions, rocketModel, rocketOptions);
}

string SimParams::getControlSystem()
{
    return controlSystem;
}

array<string, NUM_CONTROL_OPTIONS> SimParams::getControlOptions()
{
    return controlOptions;
}

string SimParams::getMotionControlSystem()
{
    return motionControlSystem;
}

array<string, NUM_MOTION_CONTROL_OPTIONS> SimParams::getMotionControlOptions()
{
    return motionControlOptions;
}

string SimParams::getRocketModel()
{
    return rocketModel;
}

array<string, NUM_ROCKET_PARAMS> SimParams::getRocketOptions()
{
    return rocketOptions;
}
