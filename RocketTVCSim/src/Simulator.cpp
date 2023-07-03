#include "Simulator.h"


float degreesToRad(float degrees) {
    //converts degrees to radians
return degrees * (3.14159265358979323846 / 180);
}

void Simulator::runSimulation()
{
    ChSystemNSC sys;
    bool autoTune = false;

    PIDParams pidParamsRate = PIDParams(0.014043886485137819, 0.21359523171311495, 0.0002308463840994613, 0.1, degreesToRad(10));
    PIDParams pidParamsAngle = PIDParams(0.0058871452048812065, 0.04113289226112847, 0.00021064941436241307, 0.1, degreesToRad(20));




    RocketModel rocket = RocketModel(1, 2, 8, 20);

    rocket.addRocketModelToSystem(sys);
    sys.Set_G_acc(ChVector<>(0, 0, 0));

    Course course = Course("C:\\Users\\patma\\source\\repos\\RocketSim\\RocketSimTemplate\\RocketTVCSim\\points.csv");

    //Setup Motion Controller
    ControlSystemTuner controlSystemTuner;

    TunableControlSystem tunableControlSystem = TunableControlSystem(controlSystemTuner, pidParamsRate, pidParamsAngle);
    MotionControlSystem motionController = MotionControlSystem(tunableControlSystem, course, 50);


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

        /*
        if (autoTune):
    print("Press Enter to Start Rate Tuning")
    input()
    resetSystem()
    autoTuneController = AutoTuneController(degreesToRad(-20), chSys.GetChTime, out_step=maxDeflectionAngle,sampletime=10, lookback=80)

    while not done:
        thrust_angle_yaw = autoTuneController.output
        accumulate_Forces()
        done = autoTuneController.run(rocket_upper.GetWvel_loc().z)
        chSys.DoStepDynamics(1e-3)
        plt.pause(1e-6)

      

    def errorMap(error):
        if (np.absolute(error) < 0.000001):
            return 0
        return error
    if autoTuneController.state == AutoTuneController.STATE_SUCCEEDED:
        for rule in autoTuneController.tuning_rules:
            params = autoTuneController.get_pid_parameters(rule)
            print('rule: {0}'.format(rule))
            print('Kp: {0}'.format(params.Kp))
            print('Ki: {0}'.format(params.Ki))
            print('Kd: {0}'.format(params.Kd))
            print()
          
            
    paramsRate = autoTuneController.get_pid_parameters("ziegler-nichols")
    print("Rate Tuning complete. Press Enter to Start Angle Tuning")
    resetSystem()
    input()
    resetSystem()
    autoTuneController = AutoTuneController(degreesToRad(-20), chSys.GetChTime, out_step=degreesToRad(5),sampletime=50, lookback=1000,  noiseband=degreesToRad(0.5))

    done = False
    i = 0
    yawRatePID = pid.PID(paramsRate.Kp,paramsRate.Ki, paramsRate.Kd, setpoint=degreesToRad(0), sample_time=0.1, output_limits=(-maxDeflectionAngle,maxDeflectionAngle), error_map=errorMap)
    yawRatePID.time_func = chSys.GetChTime
    while not done:
        yawRatePID.setpoint = autoTuneController.output
        thrust_angle_yaw = yawRatePID(rocket_upper.GetWvel_loc().z)
        setpt = yawRatePID.setpoint
        accumulate_Forces()
        done = autoTuneController.run(rocket_upper.GetRot().Q_to_Euler123().z)
        chSys.DoStepDynamics(1e-3)
        plt.pause(1e-6)
  
    if autoTuneController.state == AutoTuneController.STATE_SUCCEEDED:
        for rule in autoTuneController.tuning_rules:
            params = autoTuneController.get_pid_parameters(rule)
            print('rule: {0}'.format(rule))
            print('Kp: {0}'.format(params.Kp))
            print('Ki: {0}'.format(params.Ki))
            print('Kd: {0}'.format(params.Kd))
            print()
    #Do AutoTune of anglePID
    paramsAngle = autoTuneController.get_pid_parameters("ziegler-nichols")

    print("Angle Tuning complete. Press Enter to Start Simulation")
    pidTuneDf = pd.DataFrame({"time": xdata, "yawRate": ydata, "thrustAngle": ydata2, "yawAngle": ydata3, "yawRateGoalAngle": ydata4})
    pidTuneDf.to_csv("data2.csv")
    resetSystem()
    input()
    resetSystem()
        
        */




        tunableControlSystem.tune();
    }

    // 5 - Simulation loop
   // ChRealtimeStepTimer realtime_timer;
    double step_size = 5e-3;
    ThrustParameters thrustParameters = ThrustParameters(0, 0, 25);
    MotionCommand motionCommand;

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
        motionCommand = motionController.getNextMotionCommand(rocket.getGLocation(), rocket.getRocketUpper(), sys.GetChTime());
        thrustParameters = motionCommand.getThrustParameters();
        //Save Debug Data

        // Spin in place to maintain soft real-time
        //realtime_timer.Spin(step_size);
    }
}

Simulator::Simulator()
{
}
