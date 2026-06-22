#include "a320_fcc.h"

namespace A320 {

FlightControlComputer::FlightControlComputer()
    : _current_mode(AFMode::OFF),
      _target_altitude(0.0f), _target_vs(0.0f),
      _target_speed(250.0f), _target_mach(0.78f),
      _target_heading(0.0f), _target_pitch(0.0f),
      _pitch_integral(0.0f), _vs_integral(0.0f), _speed_integral(0.0f)
{
    _gains = {
        2.0f, 0.1f, 0.5f,   // pitch
        1.5f, 0.05f, 0.3f,  // roll
        0.01f, 0.001f,      // vs
        0.001f, 0.0001f,    // alt
        0.05f, 0.005f       // speed
    };
}

void FlightControlComputer::initialize() {
    _pitch_integral = 0.0f;
    _vs_integral = 0.0f;
    _speed_integral = 0.0f;
    _go_around.reset();
}

void FlightControlComputer::setAFMode(AFMode mode) {
    _current_mode = mode;
}

void FlightControlComputer::setTargetAltitude(float altitude_ft) {
    _target_altitude = altitude_ft;
}

void FlightControlComputer::setTargetVerticalSpeed(float vs_fpm) {
    _target_vs = vs_fpm;
}

void FlightControlComputer::setTargetSpeed(float speed_knots) {
    _target_speed = speed_knots;
}

void FlightControlComputer::setTargetMach(float mach) {
    _target_mach = mach;
}

void FlightControlComputer::setTargetHeading(float heading_deg) {
    _target_heading = heading_deg;
}

void FlightControlComputer::activateGoAround(const AircraftState& state) {
    _go_around.activate(state);
    _current_mode = AFMode::GO_AROUND;
}

ControlCommand FlightControlComputer::computeControl(const AircraftState& state) {
    ControlCommand cmd = {};

    switch (_current_mode) {
        case AFMode::OFF:
            cmd.pitch_cmd = state.pitch;
            cmd.roll_cmd = 0.0f;
            break;

        case AFMode::ATTITUDE_HOLD:
            cmd.pitch_cmd = attitudeHoldControl(state);
            cmd.roll_cmd = 0.0f;
            break;

        case AFMode::HEADING_HOLD:
            cmd.pitch_cmd = attitudeHoldControl(state);
            cmd.roll_cmd = headingHoldControl(state);
            break;

        case AFMode::ALTITUDE_HOLD:
            cmd.pitch_cmd = altitudeHoldControl(state);
            cmd.roll_cmd = headingHoldControl(state);
            break;

        case AFMode::VERTICAL_SPEED:
            cmd.pitch_cmd = verticalSpeedControl(state);
            cmd.roll_cmd = headingHoldControl(state);
            break;

        case AFMode::SPEED_HOLD:
            cmd.pitch_cmd = speedHoldControl(state);
            cmd.roll_cmd = headingHoldControl(state);
            break;

        case AFMode::MACH_HOLD:
            cmd.pitch_cmd = speedHoldControl(state);
            cmd.roll_cmd = headingHoldControl(state);
            break;

        case AFMode::CLIMB:
            cmd.pitch_cmd = verticalSpeedControl(state);
            cmd.roll_cmd = headingHoldControl(state);
            break;

        case AFMode::DESCENT:
            cmd.pitch_cmd = verticalSpeedControl(state);
            cmd.roll_cmd = headingHoldControl(state);
            break;

        case AFMode::GO_AROUND:
            cmd.pitch_cmd = goAroundControl(state);
            cmd.roll_cmd = _go_around.getRollCommand();
            break;
    }

    cmd.pitch_cmd = limitPitch(cmd.pitch_cmd);
    cmd.roll_cmd = limitRoll(cmd.roll_cmd);

    return cmd;
}

float FlightControlComputer::limitPitch(float pitch_rad) const {
    return Utils::constrain(pitch_rad, -20.0f * DEG_TO_RAD, 30.0f * DEG_TO_RAD);
}

float FlightControlComputer::limitRoll(float roll_rad) const {
    return Utils::constrain(roll_rad, -30.0f * DEG_TO_RAD, 30.0f * DEG_TO_RAD);
}

float FlightControlComputer::altitudeHoldControl(const AircraftState& state) {
    float current_alt_ft = state.alt_std / 0.3048f;
    float error = _target_altitude - current_alt_ft;

    float dt = 0.05f;

    _vs_integral += error * dt;
    _vs_integral = Utils::constrain(_vs_integral, -1000.0f, 1000.0f);

    float target_vs = _gains.alt_kp * error + _gains.alt_ki * _vs_integral;
    target_vs = Utils::constrain(target_vs, -2000.0f, 3000.0f);

    float vz_mps = std::sin(state.pitch) * state.tas;
    float current_vs_fpm = vz_mps / 0.3048f * 60.0f;
    float vs_error = target_vs - current_vs_fpm;

    _pitch_integral += vs_error * dt;
    _pitch_integral = Utils::constrain(_pitch_integral, -500.0f, 500.0f);

    float pitch_cmd = _gains.vs_kp * vs_error + _gains.vs_ki * _pitch_integral;

    return limitPitch(pitch_cmd * DEG_TO_RAD);
}

float FlightControlComputer::verticalSpeedControl(const AircraftState& state) {
    float vz_mps = std::sin(state.pitch) * state.tas;
    float current_vs_fpm = vz_mps / 0.3048f * 60.0f;
    float error = _target_vs - current_vs_fpm;

    float dt = 0.05f;

    _vs_integral += error * dt;
    _vs_integral = Utils::constrain(_vs_integral, -5000.0f, 5000.0f);

    float pitch_cmd = _gains.vs_kp * error + _gains.vs_ki * _vs_integral;

    return limitPitch(pitch_cmd * DEG_TO_RAD);
}

float FlightControlComputer::speedHoldControl(const AircraftState& state) {
    float current_speed = state.cas * MS_TO_KNOTS;
    float error = _target_speed - current_speed;

    float dt = 0.05f;

    _speed_integral += error * dt;
    _speed_integral = Utils::constrain(_speed_integral, -500.0f, 500.0f);

    float pitch_cmd = _gains.speed_kp * error + _gains.speed_ki * _speed_integral;

    return limitPitch(pitch_cmd * DEG_TO_RAD);
}

float FlightControlComputer::attitudeHoldControl(const AircraftState& state) {
    static float stored_pitch = 0.0f;
    static bool pitch_stored = false;

    if (!pitch_stored || _current_mode != AFMode::ATTITUDE_HOLD) {
        stored_pitch = state.pitch;
        pitch_stored = true;
    }

    float target_pitch = stored_pitch;
    float error = target_pitch - state.pitch;

    float dt = 0.05f;
    _pitch_integral += error * dt;
    _pitch_integral = Utils::constrain(_pitch_integral, -0.5f, 0.5f);

    float pitch_cmd = _gains.pitch_kp * error +
                      _gains.pitch_ki * _pitch_integral -
                      _gains.pitch_kd * state.pitch_rate;

    return limitPitch(pitch_cmd);
}

float FlightControlComputer::headingHoldControl(const AircraftState& state) {
    float heading_error = Utils::wrapAngle(_target_heading - state.yaw * RAD_TO_DEG);
    float target_roll = heading_error * 2.0f;

    target_roll = Utils::constrain(target_roll, -25.0f, 25.0f);

    float error = target_roll * DEG_TO_RAD - state.roll;

    float roll_cmd = _gains.roll_kp * error;

    return limitRoll(roll_cmd);
}

float FlightControlComputer::goAroundControl(const AircraftState& state) {
    static constexpr float dt = 1.0f / 50.0f;
    _go_around.update(state, dt);
    return _go_around.getPitchCommand();
}

} // namespace A320