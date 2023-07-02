#include "Simulator.h"

void Simulator::runSimulation()
{
    ChSystemNSC sys;
    bool autoTune = false;

    PIDParams pidParamsRate = PIDParams(0.1, 0.1, 0.1);
    PIDParams pidParamsAngle = PIDParams(0.1, 0.1, 0.1);




    RocketModel rocket = RocketModel(0.5, 1, 1, 10);

    rocket.addRocketModelToSystem(sys);
    sys.Set_G_acc(ChVector<>(0, 0, 0));


    //Setup Motion Controller
    ControlSystemTuner controlSystemTuner;

    TunableControlSystem tunableControlSystem = TunableControlSystem(controlSystemTuner, pidParamsRate, pidParamsAngle);
    MotionControlSystem motionController = MotionControlSystem(tunableControlSystem);


// 3 - Create the Irrlicht application and set-up the camera.

    ChVisualSystemIrrlicht vis;
    vis.AttachSystem(&sys);
    vis.SetWindowSize(1024, 768);
    vis.SetWindowTitle("Rocket Visualization Demo");
    vis.Initialize();
    vis.AddSkyBox();
    vis.AddCamera(ChVector<>(10, 10, 10));
    vis.AddTypicalLights();
    vis.EnableBodyFrameDrawing(true);
    vis.SetSymbolScale(10);
    vis.ShowInfoPanel(true);


    if (autoTune) {
        tunableControlSystem.tune();
    }

    // 5 - Simulation loop
    ChRealtimeStepTimer realtime_timer;
    double step_size = 5e-3;
    ThrustParameters thrustParameters = ThrustParameters(0, 0, 5);
    MotionCommand motionCommand;
    Course course = Course("C:\\Users\\patma\\source\\repos\\RocketSim\\RocketSimTemplate\\RocketTVCSim\\points.csv");

    while (vis.Run()) {
        //Accumulate Forces
        rocket.accumulateForces(thrustParameters.convertToForceVector());
        // Render scene
        vis.BeginScene();
        vis.Render();

        //Draw Forces

        for (ForceApplication force : rocket.getDisplayedForces()) {
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
        sys.DoStepDynamics(step_size);

        //Advance Motion Controller
        motionCommand = motionController.getNextMotionCommand(rocket.getGLocation(), rocket.getRocketUpper());
        thrustParameters = motionCommand.getThrustParameters();
        //Save Debug Data

        // Spin in place to maintain soft real-time
        realtime_timer.Spin(step_size);
    }
}

Simulator::Simulator()
{
}
