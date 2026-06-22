#pragma once

#include "a320_autopilot_common.h"

namespace A320 {

class Autothrottle {
public:
    Autothrottle();

    void setMode(bool enabled);
    bool isEnabled() const { return _enabled; }

    void setTargetSpeed(float speed_knots);
    void setTargetMach(float mach);
    void setTargetThrust(float n1_percent);
    void setTargetEPR(float epr);

    void setFlightPhase(FlightPhase phase);

    float computeThrottle(const AircraftState& state);

private:
    bool _enabled;
    float _target_speed;
    float _target_mach;
    float _target_thrust;
    float _target_epr;

    FlightPhase _flight_phase;

    struct PID {
        float kp;
        float ki;
        float kd;
        float integral;
        float max_integral;
        float min_output;
        float max_output;
    };

    PID _speed_pid;
    PID _mach_pid;
    PID _thrust_pid;

    float _last_speed_error;
    float _last_time;

    float computeSpeedControl(const AircraftState& state);
    float computeMachControl(const AircraftState& state);
    float computeThrustControl(const AircraftState& state);
};

} // namespace A320