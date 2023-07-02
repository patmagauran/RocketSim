#include "MotionControlSystem.h"
#include "TrajectoryCommand.h"

MotionControlSystem::MotionControlSystem(ControlSystem controlSystem) : controlSystem(controlSystem)
{
}

MotionCommand MotionControlSystem::getNextMotionCommand(ChVector<> g_location, std::shared_ptr<ChBody> rocket_upper)
{

	/*
	        currPos = gPos
        curVel = rocketUpper.GetPos_dt()
        if (curVel.Length() < 0.05):
            curVel = chrono.ChVectorD.GetNormalized(rocketUpper.GetPos() - currPos)
        nextCommand = self._getNextCommand(currPos, curVel)


        rotation = rocketUpper.GetRot().Q_to_Euler123()

        self.yawAnglePID.setpoint = -nextCommand.yawAngle
        self.pitchAnglePID.setpoint = -nextCommand.pitchAngle

        yawAngleO = self.yawAnglePID(rotation.z)
        self.yawRatePID.setpoint = yawAngleO
        
        yawThrustAng = self.yawRatePID(rocketUpper.GetWvel_loc().z)


        pitchAngleO = self.pitchAnglePID(rotation.x)
        self.pitchRatePID.setpoint = pitchAngleO
        # print("pitchAngleO",pitchAngleO, "pitchAngle",rotation.x)
        pitchThrustAng = self.pitchRatePID(rocketUpper.GetWvel_loc().x)
        */


    ChVector<> currentVelocity = rocket_upper->GetPos_dt();
    if (currentVelocity.Length() < 0.05)
	{
		currentVelocity = (rocket_upper->GetPos() - g_location).GetNormalized();
	}
    //MotionCommand nextCommand = getNextCommand(g_location, currentVelocity);
    TrajectoryCommand nextCommand = TrajectoryCommand(0.12, 0.2);
	ChVector<> rotation = rocket_upper->GetRot().Q_to_Euler123();


    float yawAngleO = this->controlSystem.getYawAngle(nextCommand.yawAngle, rotation.z());

    float yawThrustAng = this->controlSystem.getYawRate(yawAngleO, rocket_upper->GetWvel_loc().z());

float pitchAngleO = this->controlSystem.getPitchAngle(nextCommand.pitchAngle, rotation.x());

	float pitchThrustAng = this->controlSystem.getPitchRate(pitchAngleO, rocket_upper->GetWvel_loc().x());

	//return MotionCommand(ThrustParameters(0,0,0));
	return MotionCommand(ThrustParameters(pitchThrustAng,yawThrustAng,5));
}
