#include "a320_goaround.h"

namespace A320 {

GoAroundController::GoAroundController()
    : _is_active(false), _time_since_activation(0.0f),
      _phase(GoAroundPhase::INITIAL_ROTATION),
      _initial_pitch_rate(3.0f), _target_vs(0.0f),
      _target_speed(150.0f), _target_pitch(15.0f * DEG_TO_RAD),
      _target_heading(0.0f),
      _pitch_cmd(0.0f), _roll_cmd(0.0f),
      _allow_flap_retraction(false), _allow_gear_retraction(false)
{
    _pitch_controller = {1.5f, 0.3f, 0.2f, 0.0f, 0.0f};
    _roll_controller = {1.0f, 0.1f, 0.15f, 0.0f, 0.0f};
}

void GoAroundController::activate(const AircraftState& state) {
    _is_active = true;
    _time_since_activation = 0.0f;
    _phase = GoAroundPhase::INITIAL_ROTATION;
    _target_heading = state.yaw * RAD_TO_DEG;

    float current_speed_knots = state.cas * MS_TO_KNOTS;
    _target_speed = std::max(current_speed_knots, 150.0f);
    _target_pitch = state.pitch;
    _target_vs = 2000.0f;
    _initial_altitude_ft = state.hagl > 0.0f ? state.hagl / 0.3048f : state.alt_msl / 0.3048f;

    _pitch_controller.integral = 0.0f;
    _pitch_controller.last_error = 0.0f;
    _roll_controller.integral = 0.0f;
    _roll_controller.last_error = 0.0f;

    _pitch_cmd = state.pitch;
    _roll_cmd = state.roll;

    _allow_flap_retraction = false;
    _allow_gear_retraction = false;
    _positive_climb_established = false;
}

void GoAroundController::reset() {
    _is_active = false;
    _time_since_activation = 0.0f;
    _phase = GoAroundPhase::INITIAL_ROTATION;
}

void GoAroundController::update(const AircraftState& state, float dt) {
    if (!_is_active) return;

    _time_since_activation += dt;
    updatePhase(state);
    _pitch_cmd = computePitch(state, dt);
    _roll_cmd = computeRoll(state, dt);
}

const char* GoAroundController::getPhaseName() const {
    switch (_phase) {
        case GoAroundPhase::INITIAL_ROTATION: return "Initial Rotation";
        case GoAroundPhase::CLIMB_OUT_V2: return "Climb Out (V2+10)";
        case GoAroundPhase::ACCELERATION: return "Acceleration";
        case GoAroundPhase::CLIMB_OUT: return "Climb Out";
        case GoAroundPhase::TRANSITION_TO_CLIMB: return "Transition to Climb";
        default: return "Unknown";
    }
}

void GoAroundController::updatePhase(const AircraftState& state) {
    float altitude_ft = (state.hagl > 0.0f) ? (state.hagl / 0.3048f) : (state.alt_msl / 0.3048f);

    float vz_mps = std::sin(state.pitch) * state.tas;
    float vz_fpm = vz_mps / 0.3048f * 60.0f;
    if (vz_fpm > 100.0f) {
        _positive_climb_established = true;
    }

    switch (_phase) {
        case GoAroundPhase::INITIAL_ROTATION: {
            if (_time_since_activation > 3.0f || state.pitch > 10.0f * DEG_TO_RAD) {
                _phase = GoAroundPhase::CLIMB_OUT_V2;
            }
            break;
        }

        case GoAroundPhase::CLIMB_OUT_V2: {
            if (altitude_ft > 400.0f) {
                _phase = GoAroundPhase::ACCELERATION;
                _allow_flap_retraction = true;
            }
            break;
        }

        case GoAroundPhase::ACCELERATION: {
            if (altitude_ft > 1500.0f) {
                _phase = GoAroundPhase::CLIMB_OUT;
                _allow_gear_retraction = true;
                _target_speed = 220.0f;
            } else {
                float t = (altitude_ft - 400.0f) / 1100.0f;
                t = Utils::constrain(t, 0.0f, 1.0f);
                _target_speed = 150.0f + 70.0f * t;
            }
            break;
        }

        case GoAroundPhase::CLIMB_OUT: {
            if (altitude_ft > 3000.0f) {
                _phase = GoAroundPhase::TRANSITION_TO_CLIMB;
                _target_speed = 250.0f;
                _target_vs = 1500.0f;
            } else {
                float t = (altitude_ft - 1500.0f) / 1500.0f;
                t = Utils::constrain(t, 0.0f, 1.0f);
                _target_speed = 220.0f + 30.0f * t;
            }
            break;
        }

        case GoAroundPhase::TRANSITION_TO_CLIMB:
            break;
    }
}

float GoAroundController::computePitch(const AircraftState& state, float dt) {
    float target_pitch = computeTargetPitch(state);
    float pitch_error = target_pitch - state.pitch;

    float proportional = _pitch_controller.kp * pitch_error;

    _pitch_controller.integral += pitch_error * dt;
    _pitch_controller.integral = Utils::constrain(
        _pitch_controller.integral, -10.0f * DEG_TO_RAD, 10.0f * DEG_TO_RAD);
    float integral = _pitch_controller.ki * _pitch_controller.integral;

    float derivative = 0.0f;
    if (dt > 0.001f) {
        derivative = _pitch_controller.kd * (pitch_error - _pitch_controller.last_error) / dt;
    }
    _pitch_controller.last_error = pitch_error;

    float pitch_increment = proportional + integral + derivative;

    float max_pitch_rate_deg_s = 3.0f;
    pitch_increment = Utils::limitRate(0.0f, pitch_increment, max_pitch_rate_deg_s * DEG_TO_RAD, dt);

    float pitch_cmd = state.pitch + pitch_increment;
    pitch_cmd = limitPitchForGoAround(pitch_cmd, state);

    return pitch_cmd;
}

float GoAroundController::computeRoll(const AircraftState& state, float dt) {
    float heading_error = Utils::wrapAngle(_target_heading - state.yaw * RAD_TO_DEG);
    float target_roll = Utils::constrain(heading_error * 2.0f, -25.0f, 25.0f) * DEG_TO_RAD;
    float roll_error = target_roll - state.roll;

    float proportional = _roll_controller.kp * roll_error;

    _roll_controller.integral += roll_error * dt;
    _roll_controller.integral = Utils::constrain(
        _roll_controller.integral, -10.0f * DEG_TO_RAD, 10.0f * DEG_TO_RAD);
    float integral = _roll_controller.ki * _roll_controller.integral;

    float derivative = 0.0f;
    if (dt > 0.001f) {
        derivative = _roll_controller.kd * (roll_error - _roll_controller.last_error) / dt;
    }
    _roll_controller.last_error = roll_error;

    float roll_increment = proportional + integral + derivative;

    float max_roll_rate_deg_s = 5.0f;
    roll_increment = Utils::limitRate(0.0f, roll_increment, max_roll_rate_deg_s * DEG_TO_RAD, dt);

    float roll_cmd = state.roll + roll_increment;
    float max_roll = 25.0f * DEG_TO_RAD;
    roll_cmd = Utils::constrain(roll_cmd, -max_roll, max_roll);

    return roll_cmd;
}

float GoAroundController::computeTargetPitch(const AircraftState& state) {
    float current_speed_knots = state.cas * MS_TO_KNOTS;
    float speed_error = _target_speed - current_speed_knots;

    float groundspeed_knots = state.groundspeed * MS_TO_KNOTS;
    if (groundspeed_knots < 50.0f) groundspeed_knots = 50.0f;
    float target_fpa = std::atan2(_target_vs / 60.0f, groundspeed_knots * KNOTS_TO_MS * 3.281f);
    float base_pitch = target_fpa + state.aoa;

    float target_pitch;

    switch (_phase) {
        case GoAroundPhase::INITIAL_ROTATION: {
            float time_factor = std::min(_time_since_activation / 3.0f, 1.0f);
            target_pitch = state.pitch + time_factor * (15.0f * DEG_TO_RAD - state.pitch);
            if (state.aoa > state.alpha_prot * 0.9f) {
                target_pitch -= 2.0f * DEG_TO_RAD;
            }
            break;
        }

        case GoAroundPhase::CLIMB_OUT_V2: {
            float current_vs_fpm = std::sin(state.pitch) * state.tas / 0.3048f * 60.0f;
            float vs_error = _target_vs - current_vs_fpm;
            float pitch_for_vs = computePitchForVS(vs_error);
            float pitch_for_speed = computePitchForSpeed(speed_error);

            if (speed_error > 10.0f) {
                target_pitch = blendPitchCommands(pitch_for_vs, pitch_for_speed, speed_error);
            } else {
                target_pitch = std::max(base_pitch, pitch_for_vs);
            }
            target_pitch = std::min(target_pitch, 15.0f * DEG_TO_RAD);
            break;
        }

        case GoAroundPhase::ACCELERATION: {
            float current_vs_fpm = std::sin(state.pitch) * state.tas / 0.3048f * 60.0f;
            float vs_error = _target_vs - current_vs_fpm;
            float pitch_for_vs = computePitchForVS(vs_error);
            target_pitch = pitch_for_vs;
            if (speed_error > 5.0f) {
                target_pitch -= (speed_error - 5.0f) * 0.1f * DEG_TO_RAD;
            }
            target_pitch = std::min(target_pitch, 15.0f * DEG_TO_RAD);
            break;
        }

        case GoAroundPhase::CLIMB_OUT:
        case GoAroundPhase::TRANSITION_TO_CLIMB: {
            float current_vs_fpm = std::sin(state.pitch) * state.tas / 0.3048f * 60.0f;
            float vs_error = _target_vs - current_vs_fpm;
            float pitch_for_vs = computePitchForVS(vs_error);
            float pitch_for_speed = computePitchForSpeed(speed_error);
            target_pitch = blendPitchCommands(pitch_for_vs, pitch_for_speed, speed_error);
            target_pitch = Utils::constrain(target_pitch, -5.0f * DEG_TO_RAD, 15.0f * DEG_TO_RAD);
            break;
        }

        default:
            target_pitch = base_pitch;
    }

    return target_pitch;
}

float GoAroundController::computePitchForSpeed(float speed_error_knots) {
    float pitch_correction;
    if (speed_error_knots > 0) {
        pitch_correction = -std::min(speed_error_knots * 0.3f, 10.0f) * DEG_TO_RAD;
    } else {
        pitch_correction = std::min(-speed_error_knots * 0.2f, 5.0f) * DEG_TO_RAD;
    }
    return pitch_correction;
}

float GoAroundController::computePitchForVS(float vs_error_fpm) {
    float pitch_correction = (vs_error_fpm / 100.0f) * 1.0f * DEG_TO_RAD;
    pitch_correction = Utils::constrain(pitch_correction, -10.0f * DEG_TO_RAD, 10.0f * DEG_TO_RAD);
    return pitch_correction;
}

float GoAroundController::computePitchForAltitude(float altitude_error_ft) {
    float pitch_correction = (altitude_error_ft / 100.0f) * 1.0f * DEG_TO_RAD;
    pitch_correction = Utils::constrain(pitch_correction, -10.0f * DEG_TO_RAD, 10.0f * DEG_TO_RAD);
    return pitch_correction;
}

float GoAroundController::blendPitchCommands(float pitch_for_vs, float pitch_for_speed, float speed_error) {
    float abs_error = std::abs(speed_error);
    float vs_weight, speed_weight;

    if (abs_error < 5.0f) {
        vs_weight = 0.9f;
        speed_weight = 0.1f;
    } else if (abs_error > 20.0f) {
        vs_weight = 0.2f;
        speed_weight = 0.8f;
    } else {
        float t = (abs_error - 5.0f) / 15.0f;
        vs_weight = 0.9f - 0.7f * t;
        speed_weight = 0.1f + 0.7f * t;
    }

    return vs_weight * pitch_for_vs + speed_weight * pitch_for_speed;
}

float GoAroundController::limitPitchForGoAround(float pitch, const AircraftState& state) {
    float max_pitch_up = 15.0f;
    if (_phase == GoAroundPhase::INITIAL_ROTATION) {
        max_pitch_up = 18.0f;
    }

    float max_pitch_down = 5.0f;

    if (state.alpha_prot > 0) {
        float aoa_limit = state.alpha_prot - 2.0f * DEG_TO_RAD;
        float max_pitch_for_aoa = state.pitch - (state.aoa - aoa_limit);
        max_pitch_up = std::min(max_pitch_up, max_pitch_for_aoa * RAD_TO_DEG);
    }

    float current_speed_knots = state.cas * MS_TO_KNOTS;
    if (current_speed_knots < 140.0f) {
        max_pitch_up = std::min(max_pitch_up, 12.0f);
    }

    float pitch_deg = pitch * RAD_TO_DEG;
    pitch_deg = Utils::constrain(pitch_deg, -max_pitch_down, max_pitch_up);

    return pitch_deg * DEG_TO_RAD;
}

float GoAroundController::limitPitchRate(float pitch_rate, float max_rate_deg_s) {
    float max_rate = max_rate_deg_s * DEG_TO_RAD;
    return Utils::constrain(pitch_rate, -max_rate, max_rate);
}

} // namespace A320