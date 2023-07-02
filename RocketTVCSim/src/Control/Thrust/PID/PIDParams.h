#pragma once
class PIDParams
{
	float kp, ki, kd;
public:
PIDParams(float kp, float ki, float kd);
	float getKp();
	float getKi();
	float getKd();
};

