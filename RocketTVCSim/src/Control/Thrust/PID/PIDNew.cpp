#include "PIDNew.h"
#include <cstddef>

PIDNew::PIDNew(double kp, double ki, double kd, double sample_time, double maxOut) :
	kp(kp), ki(ki), kd(kd), sample_time(sample_time), maxOut(maxOut)
{
}

double clamp(double inputVal, double maxVal) {
	//Clamps inputVal to between -maxVal and maxVal
if (inputVal > maxVal) {
		return maxVal;
	}
	else if (inputVal < -maxVal) {
		return -maxVal;
	}
	else {
		return inputVal;
	}
}

double PIDNew::update(double input, double currentTime)
{
	/*
	   """
		Update the PID controller.

		Call the PID controller with *input_* and calculate and return a control output if
		sample_time seconds has passed since the last update. If no new output is calculated,
		return the previous output instead (or None if no value has been calculated yet).

		:param dt: If set, uses this value for timestep instead of real time. This can be used in
			simulations when simulation time is different from real time.
		"""
		if not self.auto_mode:
			return self._last_output

		now = _current_time()
		if dt is None:
			dt = now - self._last_time if (now - self._last_time) else 1e-16
		elif dt <= 0:
			raise ValueError('dt has negative value {}, must be positive'.format(dt))

		if self.sample_time is not None and dt < self.sample_time and self._last_output is not None:
			# Only update every sample_time seconds
			return self._last_output

		# Compute error terms
		error = self.setpoint - input_
		d_input = input_ - (self._last_input if (self._last_input is not None) else input_)

		# Check if must map the error
		if self.error_map is not None:
			error = self.error_map(error)

		# Compute the proportional term
		if not self.proportional_on_measurement:
			# Regular proportional-on-error, simply set the proportional term
			self._proportional = self.Kp * error
		else:
			# Add the proportional error on measurement to error_sum
			self._proportional -= self.Kp * d_input

		# Compute integral and derivative terms
		self._integral += self.Ki * error * dt
		self._integral = _clamp(self._integral, self.output_limits)  # Avoid integral windup

		self._derivative = -self.Kd * d_input / dt

		# Compute final output
		output = self._proportional + self._integral + self._derivative
		output = _clamp(output, self.output_limits)

		# Keep track of state
		self._last_output = output
		self._last_input = input_
		self._last_time = now

		return output


	*/

	double dt = currentTime - this->lastTime;
	if (dt <= 0)
	{
		dt = 1e-16;
	}
	if (this->sample_time != NULL && dt < this->sample_time && this->lastOutput != NULL)
	{
		return this->lastOutput;
	}
	double error = this->setpoint - input;
	double d_input = input - (this->lastInput != NULL ? this->lastInput : input);

	/*if (this->error_map != NULL)
	{
		error = this->error_map(error);
	}

	if (!this->proportional_on_measurement)
	*/ //{
		this->proportional = this->kp * error;
//}
	//else
	//{
		//this->proportional -= this->kp * d_input;
	//}

	this->integral += this->ki * error * dt;
	this->integral = clamp(this->integral, this->maxOut);

	this->derivative = -this->kd * d_input / dt;

	double output = this->proportional + this->integral + this->derivative;
	output = clamp(output, this->maxOut);

	this->lastOutput = output;
	this->lastInput = input;
	this->lastTime = currentTime;

	return output;
	return 0.0f;
}

void PIDNew::setSetpoint(double setpoint)
{
	this->setpoint = setpoint;
}
