/**
 * @file a320_fcc.h
 * @brief A320 飞行控制计算机
 */

#pragma once

#include "a320_autopilot_common.h"
#include "a320_goaround.h"
#include "a320_ground_control.h"
#include "a320_engine_out.h"

namespace A320 {

/**
 * @class FlightControlComputer
 * @brief 飞行控制计算机
 */
class FlightControlComputer {
public:
    FlightControlComputer();
    void initialize();

    void setAFMode(AFMode mode);
    AFMode getAFMode() const { return _current_mode; }

    void setTargetAltitude(float altitude_ft);
    void setTargetVerticalSpeed(float vs_fpm);
    void setTargetSpeed(float speed_knots);
    void setTargetMach(float mach);
    void setTargetHeading(float heading_deg);
    void setRunwayHeading(float runway_heading_deg);

    void activateGoAround(const AircraftState& state);
    void activateEngineOut(const AircraftState& state, const std::string& failed_side);

    bool isGoAroundActive() const { return _go_around.isActive(); }
    bool isEngineOutActive() const { return _engine_out.isActive(); }

    const GoAroundController& getGoAroundController() const { return _go_around; }
    const GroundController& getGroundController() const { return _ground_control; }
    const EngineOutController& getEngineOutController() const { return _engine_out; }

    ControlCommand computeControl(const AircraftState& state);

private:
    AFMode _current_mode;

    float _target_altitude;
    float _target_vs;
    float _target_speed;
    float _target_mach;
    float _target_heading;
    float _runway_heading;

    float _pitch_integral;
    float _vs_integral;
    float _speed_integral;

    GoAroundController _go_around;
    GroundController _ground_control;
    EngineOutController _engine_out;

    struct Gains {
        float pitch_kp, pitch_ki, pitch_kd;
        float roll_kp, roll_ki, roll_kd;
        float vs_kp, vs_ki;
        float alt_kp, alt_ki;
        float speed_kp, speed_ki;
    } _gains;

    float limitPitch(float pitch_rad) const;
    float limitRoll(float roll_rad) const;
    float limitRudder(float rudder_rad) const;

    float altitudeHoldControl(const AircraftState& state);
    float verticalSpeedControl(const AircraftState& state);
    float speedHoldControl(const AircraftState& state);
    float attitudeHoldControl(const AircraftState& state);
    float headingHoldControl(const AircraftState& state);
    float goAroundControl(const AircraftState& state);
    void groundTrackControl(const AircraftState& state, ControlCommand& cmd, float dt);

    void applyEngineOutCompensation(const AircraftState& state, ControlCommand& cmd, float dt);
    bool isGroundControlNeeded(const AircraftState& state) const;
};

} // namespace A320