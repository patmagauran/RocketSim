#pragma once
class PIDParams
{
	float kp, ki, kd, sampleTime, maxOutput;
public:
PIDParams(float kp, float ki, float kd, float sampleTime=0, float maxOutput=0);
	float getKp();
	float getKi();
	float getKd();
	float getSampleTime();
	float getMaxOutput();
};

