#include "Simulator.h"
#include "Control/Thrust/PID/PIDAutoTuner.h"
#include "Util/Utils.h"
#include "Control/Thrust/TunableControlSystem.h"

#include "Control/Thrust/PIDControl/PIDControlSystem.h"

#include "Control/Thrust/PIDControl/TunablePIDControlSystem.h"
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
	this->rocket = RocketModel(this->rocketParams);

	this->rocket.addRocketModelToSystem(this->sys);
	this->sys.Set_G_acc(ChVector<>(0, 0, 0));
}
double errorMap(double error) {
	return error;
}
void Simulator::runSimulation(bool autoTune)
{
	//ChSystemNSC sys;
	resetSimulator();

	/*double maxThrustAngle = degreesToRad(10);
	double maxRotationRate = degreesToRad(20);*/



	// 3 - Create the Irrlicht application and set-up the camera.




	if (autoTune) {

		tunableControlSystem->tune(this);
	}

	
	ChVisualSystemIrrlicht vis;
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

	while (vis.Run() && !DataLog::isDone()) {

		while (DataLog::isPaused()) {

		}
		//Accumulate Forces
		this->rocket.accumulateForces(this->thrustParameters.convertToForceVector());
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

		//Advance Motion Controller
		motionCommand = motionController->getNextMotionCommand(this->rocket.getGLocation(), this->rocket, sys.GetChTime());
		this->thrustParameters = motionCommand.getThrustParameters();
		//Save Debug Data

		// Spin in place to maintain soft real-time
		//realtime_timer.Spin(step_size);
	}
	cleanup();
}

void Simulator::cleanup() {
	DataLog::cleanup();

}

Simulator::Simulator(std::shared_ptr<TunableControlSystem> tunableControlSystem, std::shared_ptr<MotionControlSystem> motionControlSystem, RocketParams rocketParams)
	: thrustParameters(0, 0, 0), rocketParams(rocketParams), tunableControlSystem(tunableControlSystem), motionController(motionControlSystem), rocket(rocketParams)
{
}