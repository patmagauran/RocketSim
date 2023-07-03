#include "PIDAutoTuner.h"
#include <stdexcept>

PIDAutoTuner::PIDAutoTuner(float setPoint, float outStep, float sampleTime, float lookback, float out_min, float out_max, float noiseband, float kp, float ki, float kd, bool ratePid, float maxDeflection) : setPoint(setPoint), outStep(outStep), sampleTime(sampleTime), outMin(out_min), outMax(out_max), noiseBand(noiseband), innerParams(PIDParams(kp, ki, kd)), useInnerPID(ratePid), outerParams(0, 0, 0), state(PIDState::STATE_OFF)
{
	/*
	 if setPoint is None:
			raise ValueError('setPoint must be specified')
		# if out_step < 1:
		#     raise ValueError('out_step must be greater or equal to 1')
		if sampletime < 1:
			raise ValueError('sampletime must be greater or equal to 1')
		if lookback < sampletime:
			raise ValueError('lookback must be greater or equal to sampletime')
		if out_min >= out_max:
			raise ValueError('out_min must be less than out_max')
	*/
	if (setPoint == 0.0f)
	{
		throw std::invalid_argument("setPoint must be specified");
	}
	if (outStep < 1.0f)
	{
		throw std::invalid_argument("out_step must be greater or equal to 1");
	}
	if (sampleTime < 1.0f)
	{
		throw std::invalid_argument("sampletime must be greater or equal to 1");
	}
	if (lookback < sampleTime)
	{
		throw std::invalid_argument("lookback must be greater or equal to sampletime");
	}
	if (outMin >= outMax)
	{
		throw std::invalid_argument("out_min must be less than out_max");
	}

}

bool PIDAutoTuner::run(float input, float currentTime)
{
	/*
	# rotation = rocketUpper.GetRot().Q_to_Euler123()
		now = self._time() * 1000

		if (self._state == AutoTuneController.STATE_OFF
				or self._state == AutoTuneController.STATE_SUCCEEDED
				or self._state == AutoTuneController.STATE_FAILED):
			self._initTuner(input_val, now)
		elif (now - self._last_run_timestamp) < self._sampletime:
			return False

		self._last_run_timestamp = now

		# check input and change relay state if necessary
		if (self._state == AutoTuneController.STATE_RELAY_STEP_UP
				and input_val > self._setPoint + self._noiseband):
			self._state = AutoTuneController.STATE_RELAY_STEP_DOWN
			self._logger.debug('switched state: {0}'.format(self._state))
			self._logger.debug('input: {0}'.format(input_val))
		elif (self._state == AutoTuneController.STATE_RELAY_STEP_DOWN
				and input_val < self._setPoint - self._noiseband):
			self._state = AutoTuneController.STATE_RELAY_STEP_UP
			self._logger.debug('switched state: {0}'.format(self._state))
			self._logger.debug('input: {0}'.format(input_val))

		# set output
		if (self._state == AutoTuneController.STATE_RELAY_STEP_UP):
			self._output = self._initial_output + self._outputstep
		elif self._state == AutoTuneController.STATE_RELAY_STEP_DOWN:
			self._output = self._initial_output - self._outputstep

		# respect output limits
		self._output = min(self._output, self._out_max)
		self._output = max(self._output, self._out_min)
		if (self.ratepid):
			self.yawRatePID.setPoint = self._output
			self._output = self.yawRatePID(input_val)
		# identify peaks
		is_max = True
		is_min = True

		for val in self._inputs:
			is_max = is_max and (input_val >= val)
			is_min = is_min and (input_val <= val)

		self._inputs.append(input_val)

		# we don't want to trust the maxes or mins until the input array is full
		if len(self._inputs) < self._inputs.maxlen:
			return False

		# increment peak count and record peak time for maxima and minima
		inflection = False

		# peak types:
		# -1: minimum
		# +1: maximum
		if is_max:
			if self._peak_type == -1:
				inflection = True
			self._peak_type = 1
		elif is_min:
			if self._peak_type == 1:
				inflection = True
			self._peak_type = -1

		# update peak times and values
		if inflection:
			self._peak_count += 1
			self._peaks.append(input_val)
			self._peak_timestamps.append(now)
			self._logger.debug('found peak: {0}'.format(input_val))
			self._logger.debug('peak count: {0}'.format(self._peak_count))

		# check for convergence of induced oscillation
		# convergence of amplitude assessed on last 4 peaks (1.5 cycles)
		self._induced_amplitude = 0

		if inflection and (self._peak_count > 4):
			abs_max = self._peaks[-2]
			abs_min = self._peaks[-2]
			for i in range(0, len(self._peaks) - 2):
				self._induced_amplitude += abs(self._peaks[i] - self._peaks[i+1])
				abs_max = max(self._peaks[i], abs_max)
				abs_min = min(self._peaks[i], abs_min)

			self._induced_amplitude /= 6.0

			# check convergence criterion for amplitude of induced oscillation
			amplitude_dev = ((0.5 * (abs_max - abs_min) - self._induced_amplitude)
							 / self._induced_amplitude)

			# self._logger.debug('amplitude: {0}'.format(self._induced_amplitude))
			# self._logger.debug('amplitude deviation: {0}'.format(amplitude_dev))

			if amplitude_dev < AutoTuneController.PEAK_AMPLITUDE_TOLERANCE:
				self._state = AutoTuneController.STATE_SUCCEEDED

		# if the autotune has not already converged
		# terminate after 10 cycles
		if self._peak_count >= 20:
			self._output = 0
			self._state = AutoTuneController.STATE_FAILED
			return True

		if self._state == AutoTuneController.STATE_SUCCEEDED:
			self._output = 0

			# calculate ultimate gain
			self._Ku = 4.0 * self._outputstep / (self._induced_amplitude * math.pi)

			# calculate ultimate period in seconds
			period1 = self._peak_timestamps[3] - self._peak_timestamps[1]
			period2 = self._peak_timestamps[4] - self._peak_timestamps[2]
			self._Pu = 0.5 * (period1 + period2) / 1000.0
			return True
		return False

	*/

	// rotation = rocketUpper.GetRot().Q_to_Euler123()
	float now = currentTime;
	if (this->state == PIDState::STATE_OFF
		|| this->state == PIDState::STATE_SUCCEEDED
		|| this->state == PIDState::STATE_FAILED)
	{
		initTuner(input, now);
	}
	else if ((now - this->lastTime) < this->sampleTime)
	{
		return false;
	}
	this->lastTime = now;
	// check input and change relay state if necessary
	if (this->state == PIDState::STATE_RELAY_STEP_UP
		&& input > this->setPoint + this->noiseBand)
	{
		this->state = PIDState::STATE_RELAY_STEP_DOWN;
	}
	else if (this->state == PIDState::STATE_RELAY_STEP_DOWN
		&& input < this->setPoint - this->noiseBand)
	{
		this->state = PIDState::STATE_RELAY_STEP_UP;
	}
	// set output
	if (this->state == PIDState::STATE_RELAY_STEP_UP)
	{
		this->output = this->initialOutput + this->outStep;
	}
	else if (this->state == PIDState::STATE_RELAY_STEP_DOWN)
	{
		this->output = this->initialOutput - this->outStep;
	}
	// respect output limits
	this->output = std::min(this->output, this->outMax);
	this->output = std::max(this->output, this->outMin);
	// identify peaks
	bool isMax = true;
	bool isMin = true;
	for (int i = 0; i < this->inputs.size(); i++)
	{
		isMax = isMax && (input >= this->inputs[i]);
		isMin = isMin && (input <= this->inputs[i]);
	}
	this->inputs.push_back(input);
	// we don't want to trust the maxes or mins until the input array is full
	if (this->inputs.size() < this->inputs.max_size())
	{
		return;
	}
	// increment peak count and record peak time for maxima and minima
	bool inflection = false;
	// peak types:
	// -1: minimum
	// +1: maximum
	if (isMax)
	{
		if (this->peakType == -1)
		{
			inflection = true;
		}
		this->peakType;
	}
	else if (isMin)
	{
		if (this->peakType == 1)
		{
			inflection = true;
		}
		this->peakType = -1;
	}
	// update peak times and values
	if (inflection)
	{
		this->peakCount++;
		this->peakValues.push_back(input);
		this->peakTimestamps.push_back(now);
		/*this->logger->debug("found peak: {0}", input);
		this->logger->debug("peak count: {0}", this->peakCount);*/
	}
	// check for convergence of induced oscillation
	// convergence of amplitude assessed on last 4 peaks (1.5 cycles)
	this->inducedAmplitude = 0;
	if (inflection && (this->peakCount > 4))
	{
		float absMax = this->peakValues[this->peakValues.size() - 2];
		float absMin = this->peakValues[this->peakValues.size() - 2];
		for (int i = 0; i < this->peakValues.size() - 2; i++)
		{
			this->inducedAmplitude += abs(this->peakValues[i] - this->peakValues[i + 1]);
			absMax = std::max(this->peakValues[i], absMax);
			absMin = std::min(this->peakValues[i], absMin);
		}
		this->inducedAmplitude /= 6.0;
		// check convergence criterion for amplitude of induced oscillation
		float amplitudeDev = ((0.5 * (absMax - absMin) - this->inducedAmplitude) / this->inducedAmplitude);
		// self._logger.debug('amplitude: {0}'.format(self._induced_amplitude))
		// self._logger.debug('amplitude deviation: {0}'.format(amplitude_dev))
		if (amplitudeDev < PEAK_AMPLITUDE_TOLERANCE)
		{
			this->state = PIDState::STATE_SUCCEEDED;
		}
	}
	// if the autotune has not already converged
	// terminate after 10 cycles
	if (this->peakCount >= 20)
	{
		this->output = 0;
		this->state = PIDState::STATE_FAILED;
		return true;
	}
	if (this->state == PIDState::STATE_SUCCEEDED)
	{
		this->output = 0;
		// calculate ultimate gain
		this->Ku = 4.0 * this->outStep / (this->inducedAmplitude * 3.1415926535);
		// calculate ultimate period in seconds
		float period1 = this->peakTimestamps[3] - this->peakTimestamps[1];
		float period2 = this->peakTimestamps[4] - this->peakTimestamps[2];
		this->Pu = 0.5 * (period1 + period2) / 1000.0;
		return true;
	}
	return false;
}

/*
	def _computePID(self,


/*
	def _initTuner(self, inputValue, timestamp):


*/

void PIDAutoTuner::initTuner(float inputVal, float timestamp) {
	/*        self._peak_type = 0
		self._peak_count = 0
		self._output = 0
		self._initial_output = 0
		self._Ku = 0
		self._Pu = 0
		self._inputs.clear()
		self._peaks.clear()
		self._peak_timestamps.clear()
		self._peak_timestamps.append(timestamp)
		self._state = AutoTuneController.STATE_RELAY_STEP_UP
		*/
	this->peakType = 0;
	this->peakCount = 0;
	this->output = 0;
	this->initialOutput = 0;
	this->Ku = 0;
	this->Pu = 0;
	this->inputs.clear();
	this->peakValues.clear();
	this->peakTimestamps.clear();
	this->peakTimestamps.push_back(timestamp);
	this->state = PIDState::STATE_RELAY_STEP_UP;
}

std::map<std::string, PIDParams> PIDAutoTuner::getTuningRules()
{
	return TUNING_RULES;
}

float PIDAutoTuner::getPidIn()
{
	return 0;
}

float PIDAutoTuner::getOutput()
{
	return this->output;
}

PIDState PIDAutoTuner::getState()
{
	return this->state;
}

PIDParams PIDAutoTuner::getPidParams(std::string tuningRule = "ziegler-nichols")
{
	return TUNING_RULES.at(tuningRule);
}
