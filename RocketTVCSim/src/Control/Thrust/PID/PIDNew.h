#pragma once
class PIDNew
{
public:
    PIDNew() = default;
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
    PIDNew(double kp, double ki, double kd, double sample_time, double maxOut );

    /// Update the controller: given the current position, compute the control
    /// action.
    double update(double input, double currentTime);
    /// Set the reference/target/setpoint of the controller.
    void setSetpoint(double setpoint);
private:
    double sample_time = 1;               ///< Sampling time (seconds)
    double maxOut = 255;      ///< Maximum control output magnitude
    double kp = 1;               ///< Proportional gain
    double ki = 0;            ///< Integral gain times Ts
    double kd = 0;            ///< Derivative gain divided by Ts
    double proportional = 0;     ///< Proportional term
double integral = 0;         ///< Integral term
double derivative = 0;       ///< Derivative term
double lastTime = 0;         ///< Time of previous update
double lastOutput = 0;       ///< Control output of previous update
double lastInput = 0;        ///< Input of previous update
    double setpoint = 0;      ///< Position reference.
};

