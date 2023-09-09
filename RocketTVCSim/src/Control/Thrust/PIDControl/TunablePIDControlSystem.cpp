#include "TunablePIDControlSystem.h"

TunablePIDControlSystem::TunablePIDControlSystem( PIDParams paramsThrustAngleFromRate, PIDParams paramsRateFromAngle, std::string rateFromAngleRule, std::string thrustFromRateRule) : rateFromAngleRule(rateFromAngleRule), thrustFromRateRule(thrustFromRateRule), PIDControlSystem{ paramsThrustAngleFromRate, paramsRateFromAngle }
{

}

void TunablePIDControlSystem::tune(Simulator* sim)
{
	RocketModel* rocket = sim->getRocket();
	ChSystemNSC* sys = sim->getSystem();
	ThrustParameters thrustParameters = ThrustParameters(0, 0, rocket->getMaxThrust());
	sim->resetSimulator();
	PIDAutoTuner pidAutoTuner = PIDAutoTuner(degreesToRad(-20), rocket->getMaxThrustAngle() , 10, 50, -100000, 100000, 0.5);

	bool done = false;
	while (!done && !DataLog::isDone()) {

		while (DataLog::isPaused()) {

		}


		if (done)
			break;

		ThrustParameters thrustParameters = ThrustParameters(pidAutoTuner.getOutput(), 0, rocket->getMaxThrust());
		DataLog::logData("xAngVel", rocket->getRocketUpper()->GetWvel_loc().x());
		DataLog::logData("thrustPitch", thrustParameters.pitchAngle);
		DataLog::pushTimestamp(sys->GetChTime());

		DataLog::logData("setPoint", pidAutoTuner.getOutput());


		rocket->accumulateForces(thrustParameters.convertToForceVector());
		//	std::cout << "Wvel_loc: " << rocket->getRocketUpper()->GetWvel_loc() << std::endl;
		done = pidAutoTuner.run(rocket->getRocketUpper()->GetWvel_loc().x(), sys->GetChTime());
		sys->DoStepDynamics(1e-3);

	}
	if (DataLog::isDone()) {
		sim->cleanup();
		return;
	}
	//f->show();
	if (pidAutoTuner.getState() == PIDState::STATE_SUCCEEDED) {
		for (std::string rule : pidAutoTuner.getTuningRules()) {
			PIDParams pidParams = pidAutoTuner.getPidParams(rule);
			std::cout << "rule: " << rule << std::endl;
			std::cout << "Kp: " << pidParams.getKp() << std::endl;
			std::cout << "Ki: " << pidParams.getKi() << std::endl;
			std::cout << "Kd: " << pidParams.getKd() << std::endl;
		}
	}
	this->paramsThrustAngleFromRate = pidAutoTuner.getPidParams(thrustFromRateRule, this->paramsThrustAngleFromRate.getSampleTime(), rocket->getMaxThrustAngle());
	std::cout << "Rate Tuning complete. Press Enter to Start Angle Tuning" << std::endl;
//	std::cin.get();


	sim->resetSimulator();
	pidAutoTuner = PIDAutoTuner(degreesToRad(-20), rocket->getMaxRotationRate(), 50, 500, -100000, 10000, degreesToRad(0.5));
	done = false;
	PIDNew yawThrustAngleFromRatePID = PIDNew(this->paramsThrustAngleFromRate.getKp(), this->paramsThrustAngleFromRate.getKi(), this->paramsThrustAngleFromRate.getKd(), 0.01, rocket->getMaxThrustAngle());
	while (!done && !DataLog::isDone()) {

		while (DataLog::isPaused()) {

		}


		yawThrustAngleFromRatePID.setSetpoint(pidAutoTuner.getOutput());
		double yaw = yawThrustAngleFromRatePID.update(rocket->getRocketUpper()->GetWvel_loc().x(), sys->GetChTime());
		thrustParameters = ThrustParameters(yaw, 0, rocket->getMaxThrust());


		DataLog::logData("xAngVel", rocket->getRocketUpper()->GetWvel_loc().x());
		DataLog::logData("thrustPitch", thrustParameters.pitchAngle);
		DataLog::logData("xAngle", rocket->getRocketUpper()->GetRot().Q_to_Euler123().x());
		DataLog::logData("setPoint", pidAutoTuner.getOutput());
		DataLog::pushTimestamp(sys->GetChTime());


		rocket->accumulateForces(thrustParameters.convertToForceVector());
		done = pidAutoTuner.run(rocket->getRocketUpper()->GetRot().Q_to_Euler123().x(), sys->GetChTime());
		sys->DoStepDynamics(1e-3);



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
	if (DataLog::isDone()) {
		sim->cleanup();
		return;
	}
	//Do AutoTune of anglePID
	this->paramsRateFromAngle = pidAutoTuner.getPidParams(rateFromAngleRule,this->paramsRateFromAngle.getSampleTime(), rocket->getMaxThrustAngle());
	this->setParamsRateFromAngle(this->paramsRateFromAngle);
	this->setParamsThrustAngleFromRate(this->paramsThrustAngleFromRate);
	std::cout << "Angle Tuning complete. Press Enter to Start Simulation" << std::endl;
	sim->resetSimulator();

}
