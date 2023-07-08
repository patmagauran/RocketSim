#pragma once
class PIDParams
{
	double kp, ki, kd, sampleTime, maxOutput;
public:
PIDParams(double kp, double ki, double kd, double sampleTime=0, double maxOutput=0);
	double getKp();
	double getKi();
	double getKd();
	double getSampleTime();
	double getMaxOutput();
};

