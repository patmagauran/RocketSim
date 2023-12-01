#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "Model/RocketModel.h"

#include "Control/Thrust/ThrustParameters.h"

#include "Simulator.h"
#include "Control/Thrust/TunableControlSystem.h"

#include "Control/Thrust/PIDControl/PIDControlSystem.h"

#include "Control/Thrust/PIDControl/TunablePIDControlSystem.h"
#include "Control/Trajectory/LookaheadSystem/LookaheadMotionControlSystem.h"
#include "Util/CSVReader.h"
#include "SimSetup.h"
// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::irrlicht;


SimResults runSim(SimSetup setup, std::string name) {
	Simulator sim = Simulator();
	DataLog::initialize(name);

	//TODO: Return a runResult Object(Contains the name, Runtime, simulation runtime, reason for exitting, max distance from course, score, and distance along course)
	//TODO: Ensure error handling
	sim.setRocketParams(setup.getRocketParams());
	sim.setMotionControlSystem(setup.getMotionControlSystem());
	SimResults results = sim.runSimulation(true);
	results.distanceAlongCourse = setup.getMotionControlSystem()->getPercentComplete(results.closestPosition);
	results.name = name;
	sim.cleanup();
	return results;
}
void runDefault() {
	// Set path to Chrono data directory
	RocketParams rocketParams = RocketParams(1, 2, 8, 1, degreesToRad(40), degreesToRad(20), 5);

	// Create a Chrono physical system
	PIDParams pidParamsThrustAngleFromRate = PIDParams(0.0149925, 0.881914, 6.37183e-05, 0.01, rocketParams.getMaxThrustAngle());
	PIDParams pidParamsRateFromAngle = PIDParams(0.0328315, 0.820787, 0.000328315, 0.01, rocketParams.getMaxRotationRate());



	Course course = Course("C:\\Users\\patma\\source\\repos\\RocketSim\\RocketSimTemplate\\RocketTVCSim\\points.csv");

	//Setup Motion Controller

	std::shared_ptr<TunableControlSystem> tunableControlSystem = std::make_shared<TunablePIDControlSystem>(pidParamsThrustAngleFromRate, pidParamsRateFromAngle);
	//std::shared_ptr<ControlSystem> tunableControlSystem = std::make_shared<PIDControlSystem>(pidParamsRate, pidParamsAngle);
	std::shared_ptr < MotionControlSystem> motionController = std::make_shared <LookaheadMotionControlSystem>(tunableControlSystem, course, 25);
	Simulator sim = Simulator();
	DataLog::initialize("data");

	sim.setRocketParams(rocketParams);
	sim.setMotionControlSystem(motionController);
	sim.runSimulation(true);
	sim.cleanup();

	DataLog::initialize("data2");
	tunableControlSystem = std::make_shared<TunablePIDControlSystem>(pidParamsThrustAngleFromRate, pidParamsRateFromAngle, "tyreus-luyben");
	//std::shared_ptr<ControlSystem> tunableControlSystem = std::make_shared<PIDControlSystem>(pidParamsRate, pidParamsAngle);
	motionController = std::make_shared < LookaheadMotionControlSystem>(tunableControlSystem, course, 25);
	sim.setRocketParams(rocketParams);
	sim.setMotionControlSystem(motionController);
	sim.runSimulation(true);
	sim.cleanup();
}

void displayHelp() {
	//Should display usage args as follows:
	// 	// -f filename.csv runs from csv file
	// -h shows help
	// If no args are given, runs default
	// Otherwise args are parsed directly as follows:
	// //-c dictates the control system
	//-m dictates the motion control system
	//-o dictates the options for the control system(space separated)
	//-p dictates the options for the motion control system(space Separated)
	// -r dictates the rocket model
	// -s dictates the options for the rocket model(space separated)


	std::cout << "Usage: " << std::endl;
	std::cout << " -f filename.csv runs from csv file" << std::endl;
	std::cout << " -h shows help" << std::endl;
	std::cout << "If no args are given, runs default" << std::endl;
	std::cout << "Otherwise args are parsed directly as follows:" << std::endl;
	std::cout << " -c dictates the control system" << std::endl;
	std::cout << " -m dictates the motion control system" << std::endl;
	std::cout << " -o dictates the options for the control system(space separated)" << std::endl;
	std::cout << " -p dictates the options for the motion control system(space Separated)" << std::endl;
	std::cout << " -r dictates the rocket model" << std::endl;
	std::cout << " -s dictates the options for the rocket model(space separated)" << std::endl;
	std::cout << "Example: -c PID -o 0.01 0.1 0.001 0.2 0.3 0.4 0.5 0.6 -m Lookahead -p \"course.csv\" 25 - r Rocket - s 1 2 3 4 5 6 7 8 9" << std::endl;
	std::cout << "Would run a simulation with a PID control system with the given parameters, a lookahead motion control system with 25 lookahead, and a rocket model with the given parameters" << std::endl;
	std::cout << "Example: -f filename.csv" << std::endl;
	std::cout << "Would run a simulation with the parameters given in the csv file" << std::endl;

}

void runArgs(int argc, char* argv[]) {

	SimSetup simSetup = SimSetup::fromCLIFlags(argc, argv);
	SimResults results = runSim(simSetup, "data");
	results.printResults();
	//Pretty print the results
}


void runCsv(char* fileName) {
	CSVReader reader = CSVReader(fileName);
	std::vector<std::vector<std::string>> data = reader.getData();
	std::vector<std::string> header = data[0];

	int controlSysCol = -1;
	int motionControlCol = -1;
	int rocketModelCol = -1;
	for (int i = 0; i < header.size(); i++) {
		if (header[i] == "ControlSystem") {
			controlSysCol = i;
		}
		else if (header[i] == "MotionControlSystem") {
			motionControlCol = i;
		}
		else if (header[i] == "RocketModel") {
			rocketModelCol = i;
		}
	}
	if (controlSysCol == -1 || motionControlCol == -1 || rocketModelCol == -1) {
		std::cout << "Invalid CSV file" << std::endl;
		return;
	}
	std::ofstream resultsFile = std::ofstream("results.csv");
	resultsFile << "Name,WorldRunTime,SimRunTime,ResultType,MaxDistanceFromCourse,DistanceAlongCourse,Score" << std::endl;
	SimResults results;
	for (int i = 1; i < data.size(); i++) {
		std::vector<std::string> row = data[i];
		SimSetup simSetup = SimSetup::fromCSVRow(row, controlSysCol, motionControlCol, rocketModelCol);
		results = runSim(simSetup, "data" + std::to_string(i));
		//Save Results to CSV
		resultsFile << results.name << "," << std::to_string(results.worldRunTime) << "," << std::to_string(results.simulationRunTime) << "," << results.resultType << "," << std::to_string(results.maxDistanceFromCourse) << "," << std::to_string(results.distanceAlongCourse) << "," << std::to_string(results.score) << std::endl;
		resultsFile.flush();
	}
	resultsFile.flush();
	resultsFile.close();
}
int main(int argc, char* argv[]) {
	SetChronoDataPath(CHRONO_DATA_DIR);

	//Parse Args and run as follows:
	//-f filename.csv runs from csv file
	//-h shows help
	//other args get parsed appropriately by simsetup
	//no args means run a simple default

	if (argc > 1) {
		if (strcmp(argv[1], "-f") == 0) {
			//run from csv file
			if (argc < 3) {
				std::cout << "No file specified" << std::endl;
				return 1;
			}
			runCsv(argv[2]);
		}
		else if (strcmp(argv[1], "-h") == 0) {
			//show help
			displayHelp();
		}
		else {
			//run from args
			runArgs(argc - 1, &argv[1]);
		}
	}
	else {
		runDefault();
	}







	DataLog::cleanup(true);
	return 0;
}