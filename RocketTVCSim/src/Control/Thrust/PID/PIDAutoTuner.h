#pragma once
#include <map>
#include <string>
#include "PIDParams.h"
#include "PIDNew.h"
#include <deque>
#include <vector>
enum class PIDState
{
	STATE_OFF,
STATE_RELAY_STEP_UP,
STATE_RELAY_STEP_DOWN,
STATE_SUCCEEDED,
STATE_FAILED
};

static const    double PEAK_AMPLITUDE_TOLERANCE = 0.05;

static const std::map<std::string, PIDParams> TUNING_RULES_MAP{ 
	{"ziegler-nichols", PIDParams(34, 40, 160)},
	{ "tyreus-luyben", PIDParams(44,  9, 126) },

	{"ciancone-marlin", PIDParams(66, 88, 162)},
	{"pessen-integral", PIDParams(28, 50, 133)},
	{"some-overshoot", PIDParams(60, 40,  60)},
	{"no-overshoot", PIDParams(100, 40,  60)},
	{"brewing", PIDParams(2.5, 6, 380)}

};
static const std::vector<std::string> TUNING_RULES{
	"ziegler-nichols",
	 "tyreus-luyben",

	 "ciancone-marlin",
	 "pessen-integral",
	 "some-overshoot",
	 "no-overshoot",
	 "brewing"

};

class PIDAutoTuner
{
	PIDParams innerParams;
	PIDParams outerParams;
	bool useInnerPID;
	PIDNew innerPID;
	std::deque<double> inputs;
	double sampleTime;
	double setPoint;
	double outStep;
	double noiseBand;
	double lastTime;
	double outMin;
	double outMax;
	PIDState state;
	std::deque<double> peakTimestamps;
	std::deque<double> peakValues;
	double output;
	double peakType;
	double peakCount;
	double initialOutput;
	double inducedAmplitude;
	double Ku;
	double Pu;
	int maxInputs;
public:
	PIDAutoTuner(double setpoint, double outStep, double sampleTime, double lookback, double out_min, double out_max, double noiseband, double kp, double ki, double kd, bool ratePid, double maxDeflection);
	bool run(double input, double currentTime);
	void initTuner(double inputVal, double timestamp);
	std::vector<std::string> getTuningRules();
	double getPidIn();
	double getOutput();
	PIDState getState();
	PIDParams getPidParams(std::string tuningRule = "ziegler-nichols");
};

