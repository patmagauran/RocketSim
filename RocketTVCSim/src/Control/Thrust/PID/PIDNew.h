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
    PIDNew(float kp, float ki, float kd, float sample_time, float maxOut );

    /// Update the controller: given the current position, compute the control
    /// action.
    float update(float input, float currentTime);
    /// Set the reference/target/setpoint of the controller.
    void setSetpoint(float setpoint);
private:
    float sample_time = 1;               ///< Sampling time (seconds)
    float maxOut = 255;      ///< Maximum control output magnitude
    float kp = 1;               ///< Proportional gain
    float ki = 0;            ///< Integral gain times Ts
    float kd = 0;            ///< Derivative gain divided by Ts
    float proportional = 0;     ///< Proportional term
float integral = 0;         ///< Integral term
float derivative = 0;       ///< Derivative term
float lastTime = 0;         ///< Time of previous update
float lastOutput = 0;       ///< Control output of previous update
float lastInput = 0;        ///< Input of previous update
    float setpoint = 0;      ///< Position reference.
};

