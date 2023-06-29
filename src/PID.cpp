#include "PID.h"
#include <cmath>

/// @param  kp
///         Proportional gain
/// @param  ki
///         Integral gain
/// @param  kd
///         Derivative gain
/// @param  Ts
///         Sampling time (seconds)
/// @param  fc
///         Cutoff frequency of derivative EMA filter (Hertz),
///         zero to disable the filter entirely
PID::PID(float kp, float ki, float kd, float Ts, float f_c, float maxOutput)
    : Ts(Ts), maxOutput(maxOutput) {
    setKp(kp);
    setKi(ki);
    setKd(kd);
    setEMACutoff(f_c);
}
float PID::calcAlphaEMA(float fn) {
     if (fn <= 0)
         return 1;
    // α(fₙ) = cos(2πfₙ) - 1 + √( cos(2πfₙ)² - 4 cos(2πfₙ) + 3 )
     const float c = std::cos(2 * float(3.14159) * fn);
     return c - 1 + std::sqrt(c * c - 4 * c + 3);
    
}

/// Update the controller: given the current position, compute the control
/// action.

float PID::update(float input) {
    // The error is the difference between the reference (setpoint) and the
    // actual position (input)
    float error = setpoint - input;
    // The integral or sum of current and previous errors
    float newIntegral = integral + error;
    // Compute the difference between the current and the previous input,
    // but compute a weighted average using a factor α ∊ (0,1]
    float diff = emaAlpha * (prevInput - input);
    // Update the average
    prevInput -= diff;

    // Check if we can turn off the motor
    if (activityCount >= activityThres && activityThres) {
        float filtError = setpoint - prevInput;
        if (filtError >= -errThres && filtError <= errThres) {
            errThres = 2; // hysteresis
            integral = newIntegral;
            return 0;
        }
        else {
            errThres = 1;
        }
    }
    else {
        ++activityCount;
        errThres = 1;
    }

    bool backward = false;
    float calcIntegral = backward ? newIntegral : integral;

    // Standard PID rule
    float output = kp * error + ki_Ts * calcIntegral + kd_Ts * diff;

    // Clamp and anti-windup
    if (output > maxOutput)
        output = maxOutput;
    else if (output < -maxOutput)
        output = -maxOutput;
    else
        integral = newIntegral;

    return output;
}

void PID::setKp(float kp) { this->kp = kp; }

///< Proportional gain

void PID::setKi(float ki) { this->ki_Ts = ki * this->Ts; }

///< Integral gain

void PID::setKd(float kd) { this->kd_Ts = kd / this->Ts; }

float PID::getKp() const { return kp; }

///< Proportional gain

float PID::getKi() const { return ki_Ts / Ts; }

///< Integral gain

float PID::getKd() const { return kd_Ts * Ts; }

/// Set the cutoff frequency (-3 dB point) of the exponential moving average
/// filter that is applied to the input before taking the difference for
/// computing the derivative term.

void PID::setEMACutoff(float f_c) {
    float f_n = f_c * Ts; // normalized sampling frequency
    this->emaAlpha = f_c == 0 ? 1 : calcAlphaEMA(f_n);
}

/// Set the reference/target/setpoint of the controller.

void PID::setSetpoint(float setpoint) {
    if (this->setpoint != setpoint) this->activityCount = 0;
    this->setpoint = setpoint;
}

/// @see @ref setSetpoint(int16_t)

float PID::getSetpoint() const { return setpoint; }

/// Set the maximum control output magnitude. Default is 255, which clamps
/// the control output in [-255, +255].

void PID::setMaxOutput(float maxOutput) { this->maxOutput = maxOutput; }

/// @see @ref setMaxOutput(float)

float PID::getMaxOutput() const { return maxOutput; }

/// Reset the activity counter to prevent the motor from turning off.

void PID::resetActivityCounter() { this->activityCount = 0; }

/// Set the number of seconds after which the motor is turned off, zero to
/// keep it on indefinitely.

void PID::setActivityTimeout(float s) {
    if (s == 0)
        activityThres = 0;
    else
        activityThres = float(s / Ts) == 0 ? 1 : s / Ts;
}

/// Reset the sum of the previous errors to zero.

void PID::resetIntegral() { integral = 0; }
