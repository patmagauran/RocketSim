#include "Simulator.h"
#include "Control/Thrust/PID/PIDAutoTuner.h"


float degreesToRad(float degrees) {
	//converts degrees to radians
	return degrees * (3.14159265358979323846 / 180);
}

void Simulator::resetSimulator() {
	this->sys.Clear();
	this->rocket = RocketModel(1, 2, 8, 5);

	this->rocket.addRocketModelToSystem(this->sys);
	this->sys.Set_G_acc(ChVector<>(0, 0, 0));
}
float errorMap(float error) {
	return error;
}
void Simulator::runSimulation()
{
	//ChSystemNSC sys;
	bool autoTune = true;
	float maxDeflection = degreesToRad(10);
	float maxRotationAngle = degreesToRad(20);
	PIDParams pidParamsRate = PIDParams(0.014043886485137819, 0.21359523171311495, 0.0002308463840994613, 0.1, maxDeflection);
	PIDParams pidParamsAngle = PIDParams(0.0058871452048812065, 0.04113289226112847, 0.00021064941436241307, 0.1, maxRotationAngle);



	resetSimulator();


	Course course = Course("C:\\Users\\patma\\source\\repos\\RocketSim\\RocketSimTemplate\\RocketTVCSim\\points.csv");

	//Setup Motion Controller
	ControlSystemTuner controlSystemTuner;

	TunableControlSystem tunableControlSystem = TunableControlSystem(controlSystemTuner, pidParamsRate, pidParamsAngle);
	MotionControlSystem motionController = MotionControlSystem(tunableControlSystem, course, 50);


	// 3 - Create the Irrlicht application and set-up the camera.

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

	//ThrustParameters thrustParameters = ThrustParameters(0, 0, 25);

	if (autoTune) {
		ThrustParameters thrustParameters = ThrustParameters(0, 0, 25);
		resetSimulator();
		//    autoTuneController = AutoTuneController(degreesToRad(-20), chSys.GetChTime, out_step=maxDeflectionAngle,sampletime=10, lookback=80)
		PIDAutoTuner pidAutoTuner = PIDAutoTuner(degreesToRad(-20), maxDeflection, 10, 80, -100000, 100000, 0.5, 0, 0, 0, false,10);


		bool done = false;
		while (!done) {
			ThrustParameters thrustParameters = ThrustParameters(pidAutoTuner.getOutput(), 0, 25);
			this->rocket.accumulateForces(thrustParameters.convertToForceVector());
		//	std::cout << "Wvel_loc: " << this->rocket.getRocketUpper()->GetWvel_loc() << std::endl;
			done = pidAutoTuner.run(this->rocket.getRocketUpper()->GetWvel_loc().x(), this->sys.GetChTime());
			this->sys.DoStepDynamics(1e-3);

		}
		if (pidAutoTuner.getState() == PIDState::STATE_SUCCEEDED) {
			for (std::string rule : pidAutoTuner.getTuningRules()) {
				PIDParams pidParams = pidAutoTuner.getPidParams(rule);
				std::cout << "rule: " << rule << std::endl;
				std::cout << "Kp: " << pidParams.getKp() << std::endl;
				std::cout << "Ki: " << pidParams.getKi() << std::endl;
				std::cout << "Kd: " << pidParams.getKd() << std::endl;
			}
		}
		pidParamsRate = pidAutoTuner.getPidParams("ziegler-nichols");
		std::cout << "Rate Tuning complete. Press Enter to Start Angle Tuning" << std::endl;
		resetSimulator();
	//	input();
		resetSimulator();
		pidAutoTuner = PIDAutoTuner(degreesToRad(-20), degreesToRad(5), 50, 1000, -100000, 10000, degreesToRad(0.5),0,0,0,false,10 );
		done = false;
		PIDNew yawRatePID = PIDNew(pidParamsRate.getKp(), pidParamsRate.getKi(), pidParamsRate.getKd(), 0.1, maxDeflection);
		while (!done) {
			yawRatePID.setSetpoint(pidAutoTuner.getOutput());
			float yaw = yawRatePID.update(rocket.getRocketUpper()->GetWvel_loc().x(), this->sys.GetChTime());
			thrustParameters = ThrustParameters(yaw, 0, 25);
			this->rocket.accumulateForces(thrustParameters.convertToForceVector());
			done = pidAutoTuner.run(this->rocket.getRocketUpper()->GetRot().Q_to_Euler123().z(), this->sys.GetChTime());
			this->sys.DoStepDynamics(1e-3);
		}

		if (pidAutoTuner.getState() == PIDState::STATE_SUCCEEDED) {
			for (std::string rule : pidAutoTuner.getTuningRules()) {
				PIDParams pidParams = pidAutoTuner.getPidParams(rule);
				std::cout << "rule: " << rule << std::endl;
				std::cout << "Kp: " << pidParams.getKp() << std::endl;
				std::cout << "Ki: " << pidParams.getKi() << std::endl;
				std::cout << "Kd: " << pidParams.getKd() << std::endl;
			}
		}
		//Do AutoTune of anglePID
		pidParamsAngle = pidAutoTuner.getPidParams("ziegler-nichols");

		std::cout << "Angle Tuning complete. Press Enter to Start Simulation" << std::endl;
		resetSimulator();
		//input();
		resetSimulator();




		tunableControlSystem.tune();
	}

	// 5 - Simulation loop
   // ChRealtimeStepTimer realtime_timer;
	double step_size = 5e-3;
	MotionCommand motionCommand;

	while (vis.Run()) {
		//Accumulate Forces
		this->rocket.accumulateForces(this->thrustParameters.convertToForceVector());
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
		motionCommand = motionController.getNextMotionCommand(this->rocket.getGLocation(), this->rocket.getRocketUpper(), sys.GetChTime());
		this->thrustParameters = motionCommand.getThrustParameters();
		//Save Debug Data

		// Spin in place to maintain soft real-time
		//realtime_timer.Spin(step_size);
	}
}

Simulator::Simulator() : thrustParameters(0, 0, 0), rocket(0,0,0,0) {
}
