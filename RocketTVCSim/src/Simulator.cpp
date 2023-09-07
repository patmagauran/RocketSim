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
	this->rocket = RocketModel(1, 2, 8, 1);

	this->rocket.addRocketModelToSystem(this->sys);
	this->sys.Set_G_acc(ChVector<>(0, 0, 0));
}
double errorMap(double error) {
	return error;
}
void Simulator::runSimulation()
{
	//ChSystemNSC sys;
	bool autoTune = true;
	double maxDeflection = degreesToRad(10);
	double maxRotationAngle = degreesToRad(20);
	PIDParams pidParamsRate = PIDParams(0.0149925, 0.881914, 6.37183e-05, 0.01, maxDeflection);
	PIDParams pidParamsAngle = PIDParams(0.0328315, 0.820787, 0.000328315, 0.01, maxRotationAngle);
	DataLog::initialize("data.csv");

	resetSimulator();


	Course course = Course("C:\\Users\\patma\\source\\repos\\RocketSim\\RocketSimTemplate\\RocketTVCSim\\points.csv");

	//Setup Motion Controller
	ControlSystemTuner controlSystemTuner;

	std::shared_ptr<TunableControlSystem> tunableControlSystem = std::make_shared<TunablePIDControlSystem>(controlSystemTuner, pidParamsRate, pidParamsAngle);
	//std::shared_ptr<ControlSystem> tunableControlSystem = std::make_shared<PIDControlSystem>(pidParamsRate, pidParamsAngle);
	MotionControlSystem motionController = MotionControlSystem(tunableControlSystem, course, 50);


	// 3 - Create the Irrlicht application and set-up the camera.




	if (autoTune) {
		
		//ThrustParameters thrustParameters = ThrustParameters(0, 0, rocket.getMaxThrust());
		//resetSimulator();
		//PIDAutoTuner pidAutoTuner = PIDAutoTuner(degreesToRad(-20), maxDeflection, 10, 50, -100000, 100000, 0.5, 0, 0, 0, false, 10);

		//bool done = false;
		//while (!done && !DataLog::isDone()) {

		//	while (DataLog::isPaused()) {

		//	}
		//

		//	if (done)
		//		break;
		//	
		//	ThrustParameters thrustParameters = ThrustParameters(pidAutoTuner.getOutput(), 0, rocket.getMaxThrust());
		//	DataLog::logData("xAngVel", this->rocket.getRocketUpper()->GetWvel_loc().x());
		//	DataLog::logData("thrustPitch", thrustParameters.pitchAngle);
		//	DataLog::pushTimestamp(this->sys.GetChTime());
		//	
		//	DataLog::logData("setPoint", pidAutoTuner.getOutput());
		//	

		//	this->rocket.accumulateForces(thrustParameters.convertToForceVector());
		//	//	std::cout << "Wvel_loc: " << this->rocket.getRocketUpper()->GetWvel_loc() << std::endl;
		//	done = pidAutoTuner.run(this->rocket.getRocketUpper()->GetWvel_loc().x(), this->sys.GetChTime());
		//	this->sys.DoStepDynamics(1e-3);
		//	
		//}
		//if (DataLog::isDone()) {
		//	cleanup();
		//	return;
		//}
		////f->show();
		//if (pidAutoTuner.getState() == PIDState::STATE_SUCCEEDED) {
		//	for (std::string rule : pidAutoTuner.getTuningRules()) {
		//		PIDParams pidParams = pidAutoTuner.getPidParams(rule);
		//		std::cout << "rule: " << rule << std::endl;
		//		std::cout << "Kp: " << pidParams.getKp() << std::endl;
		//		std::cout << "Ki: " << pidParams.getKi() << std::endl;
		//		std::cout << "Kd: " << pidParams.getKd() << std::endl;
		//	}
		//}
		//pidParamsRate = pidAutoTuner.getPidParams("ziegler-nichols");
		//std::cout << "Rate Tuning complete. Press Enter to Start Angle Tuning" << std::endl;
		//std::cin.get();


		//resetSimulator();
		//pidAutoTuner = PIDAutoTuner(degreesToRad(-20), degreesToRad(5), 50, 500, -100000, 10000, degreesToRad(0.5), 0, 0, 0, false, 10);
		//done = false;
		//PIDNew yawRatePID = PIDNew(pidParamsRate.getKp(), pidParamsRate.getKi(), pidParamsRate.getKd(), 0.01, maxDeflection);
		//while (!done && !DataLog::isDone()) {

		//	while (DataLog::isPaused()) {

		//	}


		//	yawRatePID.setSetpoint(pidAutoTuner.getOutput());
		//	double yaw = yawRatePID.update(rocket.getRocketUpper()->GetWvel_loc().x(), this->sys.GetChTime());
		//	thrustParameters = ThrustParameters(yaw, 0, rocket.getMaxThrust());


		//	DataLog::logData("xAngVel", this->rocket.getRocketUpper()->GetWvel_loc().x());
		//	DataLog::logData("thrustPitch", thrustParameters.pitchAngle);
		//	DataLog::logData("xAngle", this->rocket.getRocketUpper()->GetRot().Q_to_Euler123().x());
		//	DataLog::logData("setPoint", pidAutoTuner.getOutput());
		//	DataLog::pushTimestamp(this->sys.GetChTime());


		//	this->rocket.accumulateForces(thrustParameters.convertToForceVector());
		//	done = pidAutoTuner.run(this->rocket.getRocketUpper()->GetRot().Q_to_Euler123().x(), this->sys.GetChTime());
		//	this->sys.DoStepDynamics(1e-3);



		//}

		//if (pidAutoTuner.getState() == PIDState::STATE_SUCCEEDED) {
		//	for (std::string rule : pidAutoTuner.getTuningRules()) {
		//		PIDParams pidParams = pidAutoTuner.getPidParams(rule);
		//		std::cout << "rule: " << rule << std::endl;
		//		std::cout << "Kp: " << pidParams.getKp() << std::endl;
		//		std::cout << "Ki: " << pidParams.getKi() << std::endl;
		//		std::cout << "Kd: " << pidParams.getKd() << std::endl;
		//	}
		//}
		//if (DataLog::isDone()) {
		//	cleanup();
		//	return;
		//}
		////Do AutoTune of anglePID
		//pidParamsAngle = pidAutoTuner.getPidParams("ziegler-nichols");

		//std::cout << "Angle Tuning complete. Press Enter to Start Simulation" << std::endl;
		//resetSimulator();



		//tunableControlSystem->setParamsAngle(pidParamsAngle);
		//tunableControlSystem->setParamsRate(pidParamsRate);


		tunableControlSystem->tune(maxDeflection, maxRotationAngle, this);
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

		std::vector<ChVector<>> waypoints = course.getWaypoints();
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
		motionCommand = motionController.getNextMotionCommand(this->rocket.getGLocation(), this->rocket, sys.GetChTime());
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


Simulator::Simulator() : thrustParameters(0, 0, 0), rocket(0, 0, 0, 0) {
}
