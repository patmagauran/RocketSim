#pragma once
class PIDParams
{
	float kp, ki, kd, sampleTime, maxOutput;
public:
PIDParams(float kp, float ki, float kd, float sampleTime, float maxOutput);
	float getKp();
	float getKi();
	float getKd();
	float getSampleTime();
	float getMaxOutput();
};

