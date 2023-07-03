#pragma once
#include <map>
#include <string>
#include "PIDParams.h"
#include "PIDNew.h"
#include <deque>
enum class PIDState
{
	STATE_OFF,
STATE_RELAY_STEP_UP,
STATE_RELAY_STEP_DOWN,
STATE_SUCCEEDED,
STATE_FAILED
};

static const    float PEAK_AMPLITUDE_TOLERANCE = 0.05;

static const std::map<std::string, PIDParams> TUNING_RULES{ 
	{"ziegler-nichols", PIDParams(34, 40, 160)},
	{ "tyreus-luyben", PIDParams(44,  9, 126) },

	{"ciancone-marlin", PIDParams(66, 88, 162)},
	{"pessen-integral", PIDParams(28, 50, 133)},
	{"some-overshoot", PIDParams(60, 40,  60)},
	{"no-overshoot", PIDParams(100, 40,  60)},
	{"brewing", PIDParams(2.5, 6, 380)}

};
class PIDAutoTuner
{
	PIDParams innerParams;
	PIDParams outerParams;
	bool useInnerPID;
	PIDNew innerPID;
	std::deque<float> inputs;
	float sampleTime;
	float setPoint;
	float outStep;
	float noiseBand;
	float lastTime;
	float outMin;
	float outMax;
	PIDState state;
	std::deque<float> peakTimestamps;
	std::deque<float> peakValues;
	float output;
	float peakType;
	float peakCount;
	float initialOutput;
	float inducedAmplitude;
	float Ku;
	float Pu;
public:
	PIDAutoTuner(float setpoint, float outStep, float sampleTime, float lookback, float out_min, float out_max, float noiseband, float kp, float ki, float kd, bool ratePid, float maxDeflection);
	bool run(float input, float currentTime);
	void initTuner(float inputVal, float timestamp);
	std::map<std::string, PIDParams> getTuningRules();
	float getPidIn();
	float getOutput();
	PIDState getState();
	PIDParams getPidParams(std::string tuningRule = "ziegler-nichols");
};

