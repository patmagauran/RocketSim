#pragma once
/// Standard PID (proportional, integral, derivative) controller. Derivative
/// component is filtered using an exponential moving average filter.
class PID {
public:
    PID() = default;
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
    PID(float kp, float ki, float kd, float Ts, float f_c = 0,
        float maxOutput = 255);

    /// Update the controller: given the current position, compute the control
    /// action.
    float update(float input);

    void setKp(float kp);               ///< Proportional gain
    void setKi(float ki); ///< Integral gain
    void setKd(float kd); ///< Derivative gain

    float getKp() const;         ///< Proportional gain
    float getKi() const; ///< Integral gain
    float getKd() const; ///< Derivative gain

    /// Set the cutoff frequency (-3 dB point) of the exponential moving average
    /// filter that is applied to the input before taking the difference for
    /// computing the derivative term.
    void setEMACutoff(float f_c);

    /// Set the reference/target/setpoint of the controller.
    void setSetpoint(float setpoint);
    /// @see @ref setSetpoint(int16_t)
    float getSetpoint() const;

    /// Set the maximum control output magnitude. Default is 255, which clamps
    /// the control output in [-255, +255].
    void setMaxOutput(float maxOutput);
    /// @see @ref setMaxOutput(float)
    float getMaxOutput() const;

    /// Reset the activity counter to prevent the motor from turning off.
    void resetActivityCounter();
    /// Set the number of seconds after which the motor is turned off, zero to
    /// keep it on indefinitely.
    void setActivityTimeout(float s);

    /// Reset the sum of the previous errors to zero.
    void resetIntegral();

    float calcAlphaEMA(float fn);

private:
    float Ts = 1;               ///< Sampling time (seconds)
    float maxOutput = 255;      ///< Maximum control output magnitude
    float kp = 1;               ///< Proportional gain
    float ki_Ts = 0;            ///< Integral gain times Ts
    float kd_Ts = 0;            ///< Derivative gain divided by Ts
    float emaAlpha = 1;         ///< Weight factor of derivative EMA filter.
    float prevInput = 0;        ///< (Filtered) previous input for derivative.
    float activityCount = 0; ///< How many ticks since last setpoint change.
    float activityThres = 0; ///< Threshold for turning off the output.
    int errThres = 1;       ///< Threshold with hysteresis.
    __int32 integral = 0;       ///< Sum of previous errors for integral.
    float setpoint = 0;      ///< Position reference.
};

