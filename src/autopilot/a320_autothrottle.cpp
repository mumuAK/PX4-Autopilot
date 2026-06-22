#include "a320_autothrottle.h"

namespace A320 {

Autothrottle::Autothrottle()
    : _enabled(false), _target_speed(250.0f), _target_mach(0.78f),
      _target_thrust(80.0f), _target_epr(1.0f),
      _flight_phase(FlightPhase::CRUISE), _last_speed_error(0.0f), _last_time(0.0f)
{
    _speed_pid = {0.5f, 0.02f, 0.1f, 0.0f, 10.0f, 0.0f, 100.0f};
    _mach_pid = {0.8f, 0.01f, 0.15f, 0.0f, 5.0f, 0.0f, 100.0f};
    _thrust_pid = {1.0f, 0.05f, 0.2f, 0.0f, 20.0f, 0.0f, 100.0f};
}

void Autothrottle::setMode(bool enabled) {
    _enabled = enabled;
    if (!enabled) {
        _speed_pid.integral = 0.0f;
        _mach_pid.integral = 0.0f;
        _thrust_pid.integral = 0.0f;
    }
}

void Autothrottle::setTargetSpeed(float speed_knots) {
    _target_speed = speed_knots;
}

void Autothrottle::setTargetMach(float mach) {
    _target_mach = mach;
}

void Autothrottle::setTargetThrust(float n1_percent) {
    _target_thrust = n1_percent;
}

void Autothrottle::setTargetEPR(float epr) {
    _target_epr = epr;
}

void Autothrottle::setFlightPhase(FlightPhase phase) {
    _flight_phase = phase;
}

float Autothrottle::computeThrottle(const AircraftState& state) {
    if (!_enabled) {
        return 0.0f;
    }

    float throttle = 0.0f;

    switch (_flight_phase) {
        case FlightPhase::TAKEOFF:
        case FlightPhase::CLIMB:
            throttle = computeThrustControl(state);
            break;

        case FlightPhase::CRUISE:
            if (state.mach < 0.4f) {
                throttle = computeSpeedControl(state);
            } else {
                throttle = computeMachControl(state);
            }
            break;

        case FlightPhase::DESCENT:
        case FlightPhase::APPROACH:
            throttle = computeSpeedControl(state);
            break;

        case FlightPhase::GO_AROUND:
            throttle = 95.0f;
            break;

        default:
            throttle = 0.0f;
    }

    return Utils::constrain(throttle, 0.0f, 100.0f);
}

float Autothrottle::computeSpeedControl(const AircraftState& state) {
    float current_speed = state.cas * MS_TO_KNOTS;
    float error = _target_speed - current_speed;

    float dt = 0.05f;

    _speed_pid.integral += error * dt;
    _speed_pid.integral = Utils::constrain(_speed_pid.integral,
                                           -_speed_pid.max_integral,
                                           _speed_pid.max_integral);

    float derivative = (error - _last_speed_error) / dt;
    _last_speed_error = error;

    float output = _speed_pid.kp * error +
                   _speed_pid.ki * _speed_pid.integral +
                   _speed_pid.kd * derivative;

    return Utils::constrain(output + 50.0f, _speed_pid.min_output, _speed_pid.max_output);
}

float Autothrottle::computeMachControl(const AircraftState& state) {
    float error = _target_mach - state.mach;

    float dt = 0.05f;

    _mach_pid.integral += error * dt;
    _mach_pid.integral = Utils::constrain(_mach_pid.integral,
                                          -_mach_pid.max_integral,
                                          _mach_pid.max_integral);

    float output = _mach_pid.kp * error +
                   _mach_pid.ki * _mach_pid.integral;

    return Utils::constrain(output + 50.0f, _mach_pid.min_output, _mach_pid.max_output);
}

float Autothrottle::computeThrustControl(const AircraftState& state) {
    float current_thrust = (state.n1_left + state.n1_right) / 2.0f;
    float error = _target_thrust - current_thrust;

    float dt = 0.05f;

    _thrust_pid.integral += error * dt;
    _thrust_pid.integral = Utils::constrain(_thrust_pid.integral,
                                           -_thrust_pid.max_integral,
                                           _thrust_pid.max_integral);

    float output = _thrust_pid.kp * error +
                   _thrust_pid.ki * _thrust_pid.integral;

    return Utils::constrain(output + current_thrust, _thrust_pid.min_output, _thrust_pid.max_output);
}

} // namespace A320