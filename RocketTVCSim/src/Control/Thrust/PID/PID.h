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
    PID(double kp, double ki, double kd, double Ts, double f_c = 0,
        double maxOutput = 255);

    /// Update the controller: given the current position, compute the control
    /// action.
    double update(double input);

    void setKp(double kp);               ///< Proportional gain
    void setKi(double ki); ///< Integral gain
    void setKd(double kd); ///< Derivative gain

    double getKp() const;         ///< Proportional gain
    double getKi() const; ///< Integral gain
    double getKd() const; ///< Derivative gain

    /// Set the cutoff frequency (-3 dB point) of the exponential moving average
    /// filter that is applied to the input before taking the difference for
    /// computing the derivative term.
    void setEMACutoff(double f_c);

    /// Set the reference/target/setpoint of the controller.
    void setSetpoint(double setpoint);
    /// @see @ref setSetpoint(int16_t)
    double getSetpoint() const;

    /// Set the maximum control output magnitude. Default is 255, which clamps
    /// the control output in [-255, +255].
    void setMaxOutput(double maxOutput);
    /// @see @ref setMaxOutput(double)
    double getMaxOutput() const;

    /// Reset the activity counter to prevent the motor from turning off.
    void resetActivityCounter();
    /// Set the number of seconds after which the motor is turned off, zero to
    /// keep it on indefinitely.
    void setActivityTimeout(double s);

    /// Reset the sum of the previous errors to zero.
    void resetIntegral();

    double calcAlphaEMA(double fn);

private:
    double Ts = 1;               ///< Sampling time (seconds)
    double maxOutput = 255;      ///< Maximum control output magnitude
    double kp = 1;               ///< Proportional gain
    double ki_Ts = 0;            ///< Integral gain times Ts
    double kd_Ts = 0;            ///< Derivative gain divided by Ts
    double emaAlpha = 1;         ///< Weight factor of derivative EMA filter.
    double prevInput = 0;        ///< (Filtered) previous input for derivative.
    double activityCount = 0; ///< How many ticks since last setpoint change.
    double activityThres = 0; ///< Threshold for turning off the output.
    int errThres = 1;       ///< Threshold with hysteresis.
    __int32 integral = 0;       ///< Sum of previous errors for integral.
    double setpoint = 0;      ///< Position reference.
};

