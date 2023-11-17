#include "ControlState.h"
Eigen::Matrix<double, 1, -1> ControlState::convertToSpaceStateState()
{
	Eigen::Matrix<double, 1, 7> state; // = [ax, ay, az, wy, wz, ey, ez]
	//std::shared_ptr<ChBody> rocket_upper = rocket.getRocketUpper();
	//ChVector<> rocketAcc = rocket_upper->GetPos_dtdt();
	//ChVector<> rocket_euler_dt = rocket_upper->GetRot_dt().Q_to_Euler123();
	//ChVector<> rocketWacc = rocket_upper->GetWacc_loc();
	state[0] = accx;
	state[1] = accy;
	state[2] = accz;
	state[3] = waccy;
	state[4] = waccz;
	state[5] = eulerdty;
	state[6] = eulerdtz;
	return state;
	//return Eigen::Matrix<double, 1, -1>();
}

ControlState ControlState::computeCurrentStateFromModel(ChVector<> g_location, RocketModel rocket)
{
	ControlState state;
	std::shared_ptr<ChBody> rocket_upper = rocket.getRocketUpper();
	ChVector<> currentVelocity = rocket_upper->GetPos_dt();
	if (currentVelocity.Length() < 0.05)
	{
		currentVelocity = (rocket_upper->GetPos() - g_location).GetNormalized();
	}
	ChVector<> rotation = rocket_upper->GetRot().Q_to_Euler123();

	state.yawAngle = rotation.z();
	state.pitchAngle = rotation.x();
	state.wvelx = rocket_upper->GetWvel_loc().x();

	state.wvely = rocket_upper->GetWvel_loc().y();
	state.wvelz = rocket_upper->GetWvel_loc().z();
	state.velocity = currentVelocity.Length();
	state.accx = rocket_upper->GetPos_dtdt().x();
	state.accy = rocket_upper->GetPos_dtdt().y();
	state.accz = rocket_upper->GetPos_dtdt().z();
	state.waccy = rocket_upper->GetWacc_loc().y();
	state.waccz = rocket_upper->GetWacc_loc().z();
	state.eulerdty = rocket_upper->GetRot_dt().Q_to_Euler123().y();
	state.eulerdtz = rocket_upper->GetRot_dt().Q_to_Euler123().z();
	state.maxThrust = rocket.getMaxThrust();

	return state;
	//yaw correlates to rotation.z, pitch is x
	//return ControlState();
}

ControlState ControlState::computeDesiredState(double yawAngle, double pitchAngle)
{
	ControlState controlState;
	controlState.yawAngle = yawAngle;
	controlState.pitchAngle = pitchAngle;
	return controlState;
}
