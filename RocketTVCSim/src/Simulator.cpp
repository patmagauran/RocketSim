#include "Simulator.h"
#include "Control/Thrust/PID/PIDAutoTuner.h"
#include "Util/Utils.h"
#include "Control/Thrust/TunableControlSystem.h"

#include "Control/Thrust/PIDControl/PIDControlSystem.h"

#include "Control/Thrust/PIDControl/TunablePIDControlSystem.h"
#include "Control/Trajectory/NullControlSystem/NullMotionControlSystem.h"
using namespace matplot;


ChSystemNSC* Simulator::getSystem()
{
	return &this->sys;
}

RocketModel* Simulator::getRocket()
{
	return &this->rocket;
}

void Simulator::resetSimulator() {
	this->sys.Clear();
    this->sys.SetChTime(0);
	this->rocket.reset(this->rocketParams);

	this->rocket.addRocketModelToSystem(this->sys, this->vis);
	this->sys.Set_G_acc(ChVector<>(0, 0, 0));
}
double errorMap(double error) {
	return error;
}

void Simulator::initialize() {
	if (initialized) return;
	vis.AttachSystem(&this->sys);
	vis.SetWindowSize(1024, 768);
	vis.SetWindowTitle("Rocket Visualization Demo");
	vis.Initialize();
	vis.AddSkyBox();
	vis.AddCamera(ChVector<>(10, 10, 10));
	vis.AddTypicalLights();
	vis.EnableBodyFrameDrawing(true);
	vis.SetSymbolScale(10);
	vis.ShowInfoPanel(true);
	initialized = true;
}

SimResults Simulator::runSimulation(bool autoTune)
{
	SimResults results;
	try {
		//ChSystemNSC sys;
		resetSimulator();

		/*double maxThrustAngle = degreesToRad(10);
		double maxRotationRate = degreesToRad(20);*/



		// 3 - Create the Irrlicht application and set-up the camera.

		//Should add stage to datalog
		//It will start a new csv file, clear the plotUi, and reset dataLog


		if (autoTune) {

			motionController->tune(this);
		}


		initialize();
		DataLog::pushEvent(EventType::STAGE, "main");
		resetSimulator();
		// Cleanup

		//Make a sphere to represent the lookahead point
		std::shared_ptr<ChBody> sphere = std::make_shared<ChBodyEasySphere>(1, 0);
		sphere->SetPos(ChVector<>(0, 0, 0));
		sphere->SetBodyFixed(true);
		sphere->SetCollide(false);
		sys.AddBody(sphere);

		// 5 - Simulation loop
	   // ChRealtimeStepTimer realtime_timer;
		double step_size = 5e-3;
		MotionCommand motionCommand;
		std::chrono::high_resolution_clock clock;
		auto start = clock.now();
		results.resultType = RESULT_TYPE_SUCCESS;
		while (vis.Run() && !DataLog::isDone()) {

			while (DataLog::isPaused()) {

			}
			//Accumulate Forces
			this->rocket.accumulateForces(this->thrustParameters);
			this->rocket.accumulateDrag(ChVector<>(0, 0, 0));
			sphere->SetPos(motionCommand.getTrajectoryCommand().lookaheadPoint);

			// Render scene
			vis.BeginScene();
			vis.Render();

			//Draw Forces

			for (ForceApplication force : this->rocket.getDisplayedForces()) {
				tools::drawSegment(&vis, force.point, force.point - (force.force * 10));
			}
			//Draw Path

			std::vector<ChVector<>> waypoints = motionController->getWaypoints();
			ChVector<> waypoint, nextWaypoint;
			for (int i = 0; i < waypoints.size() - 1; i++) {
				waypoint = waypoints[i];
				if (i == waypoints.size() - 2) {
					nextWaypoint = waypoint;
				}
				else {
					nextWaypoint = waypoints[(i + 1)];
				}

				tools::drawSegment(&vis, waypoint, nextWaypoint);
			}
			//Draw Coordinate System
			//drawCoordsys(vis, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)), 100)
			irrlicht::tools::drawCoordsys(&vis, ChCoordsys<>(ChVector<>(0, 0, 0)), 100);
			vis.EndScene();


			// Perform the integration stpe
			this->sys.DoStepDynamics(step_size);
			//We need to compute distance from rocket to course
			// 
			rocket.logRocketData();
			ChVector<> closestPoint = motionController->getClosestPoint(this->rocket.getGLocation());
			results.closestPosition = closestPoint;
			double distance = (closestPoint - this->rocket.getGLocation()).Length();
			results.score += distance + 1;
			if (distance > results.maxDistanceFromCourse) {
				results.maxDistanceFromCourse = distance;
			}
			DataLog::logData("course_deviation", distance);
			DataLog::pushTimestamp(sys.GetChTime());
			double rotationalVelocity = rocket.getRocketUpper()->GetRot_dt().Length();
			if (distance > 50 || rotationalVelocity > 1) {
				results.resultType = RESULT_TYPE_FAIL;
				break;
			}


			//Advance Motion Controller
			motionCommand = motionController->getNextMotionCommand(this->rocket.getGLocation(), this->rocket, sys.GetChTime());
			this->thrustParameters = motionCommand.getThrustParameters();
			//Save Debug Data

			// Spin in place to maintain soft real-time
			//realtime_timer.Spin(step_size);
		}
		results.simulationRunTime = sys.GetChTime();
		
	auto end = clock.now();
std::chrono::duration<double> elapsed = end - start;
results.worldRunTime = elapsed.count();
	}
	catch (const std::exception& e) {
		std::cout << e.what() << std::endl;
		results.resultType = RESULT_TYPE_ERROR;
	}
	cleanup();
return results;
}

void Simulator::cleanup() {

	DataLog::cleanup(false);

}

Simulator::Simulator()
	: thrustParameters(0, 0, 0), rocketParams(), motionController(std::make_shared<NullMotionControlSystem>()), rocket(RocketParams()), maxDeviationFromCourse(100)
{
	

}

void Simulator::setMotionControlSystem(std::shared_ptr<MotionControlSystem> motionControlSystem) {
	this->motionController = motionControlSystem;
}
void Simulator::setRocketParams(RocketParams rocketParams) {
	this->rocketParams = rocketParams;
}

void Simulator::setMaxDeviationFromCourse(double maxDeviationFromCourse)
{
	this->maxDeviationFromCourse = maxDeviationFromCourse;
}


//There are now some display glitches. It seems that models aren't getting loaded now. And the second launch of the plot ui doesn't work. We should have a plot UI that is always running and just updates the data.