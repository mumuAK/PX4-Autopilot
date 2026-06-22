#pragma once

#include "a320_autopilot_common.h"
#include "a320_goaround.h"

namespace A320 {

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

    void activateGoAround(const AircraftState& state);
    bool isGoAroundActive() const { return _go_around.isActive(); }
    const GoAroundController& getGoAroundController() const { return _go_around; }

    ControlCommand computeControl(const AircraftState& state);

private:
    AFMode _current_mode;

    float _target_altitude;
    float _target_vs;
    float _target_speed;
    float _target_mach;
    float _target_heading;
    float _target_pitch;

    float _pitch_integral;
    float _vs_integral;
    float _speed_integral;

    GoAroundController _go_around;

    struct Gains {
        float pitch_kp;
        float pitch_ki;
        float pitch_kd;
        float roll_kp;
        float roll_ki;
        float roll_kd;
        float vs_kp;
        float vs_ki;
        float alt_kp;
        float alt_ki;
        float speed_kp;
        float speed_ki;
    } _gains;

    float limitPitch(float pitch_rad) const;
    float limitRoll(float roll_rad) const;

    float altitudeHoldControl(const AircraftState& state);
    float verticalSpeedControl(const AircraftState& state);
    float speedHoldControl(const AircraftState& state);
    float attitudeHoldControl(const AircraftState& state);
    float headingHoldControl(const AircraftState& state);
    float goAroundControl(const AircraftState& state);
};

} // namespace A320