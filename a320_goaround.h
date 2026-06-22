#pragma once

#include "a320_autopilot_common.h"

namespace A320 {

class GoAroundController {
public:
    GoAroundController();

    void activate(const AircraftState& state);
    void reset();

    void update(const AircraftState& state, float dt);

    GoAroundPhase getPhase() const { return _phase; }
    const char* getPhaseName() const;

    float getPitchCommand() const { return _pitch_cmd; }
    float getRollCommand() const { return _roll_cmd; }
    float getTargetSpeed() const { return _target_speed; }
    float getTargetVerticalSpeed() const { return _target_vs; }
    bool isFlapRetractionAllowed() const { return _allow_flap_retraction; }
    bool isGearRetractionAllowed() const { return _allow_gear_retraction && _positive_climb_established; }

    bool isActive() const { return _is_active; }
    bool isCompleted() const { return _phase == GoAroundPhase::TRANSITION_TO_CLIMB; }

private:
    bool _is_active;
    float _time_since_activation;

    GoAroundPhase _phase;
    float _initial_pitch_rate;
    float _target_vs;
    float _target_speed;
    float _target_pitch;
    float _target_heading;
    float _initial_altitude_ft;
    bool _positive_climb_established;

    float _pitch_cmd;
    float _roll_cmd;

    bool _allow_flap_retraction;
    bool _allow_gear_retraction;

    struct PitchController {
        float kp;
        float ki;
        float kd;
        float integral;
        float last_error;
    } _pitch_controller;

    struct RollController {
        float kp;
        float ki;
        float kd;
        float integral;
        float last_error;
    } _roll_controller;

    void updatePhase(const AircraftState& state);
    float computePitch(const AircraftState& state, float dt);
    float computeRoll(const AircraftState& state, float dt);
    float computeTargetPitch(const AircraftState& state);
    float computePitchForSpeed(float speed_error_knots);
    float computePitchForAltitude(float altitude_error_ft);
    float computePitchForVS(float vs_error_fpm);
    float blendPitchCommands(float pitch_for_vs, float pitch_for_speed, float speed_error);
    float limitPitchForGoAround(float pitch, const AircraftState& state);
    float limitPitchRate(float pitch_rate, float max_rate_deg_s = 5.0f);
};

} // namespace A320