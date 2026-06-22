#pragma once

#include "a320_autopilot_common.h"

namespace A320 {

class FlightManagementSystem {
public:
    FlightManagementSystem();

    void initialize();
    void setFlightPlan();

    void update(const AircraftState& state);

    float getTargetAltitude() const { return _target_altitude; }
    float getTargetSpeed() const { return _target_speed; }
    float getTargetMach() const { return _target_mach; }
    float getTargetHeading() const { return _target_heading; }
    float getTargetVS() const { return _target_vs; }

    void setTargetAltitude(float alt_ft) { _target_altitude = alt_ft; }
    void setTargetSpeed(float speed_knots) { _target_speed = speed_knots; }
    void setTargetMach(float mach) { _target_mach = mach; }
    void setTargetHeading(float heading_deg) { _target_heading = heading_deg; }
    void setTargetVS(float vs_fpm) { _target_vs = vs_fpm; }

private:
    float _target_altitude;
    float _target_speed;
    float _target_mach;
    float _target_heading;
    float _target_vs;

    float _current_waypoint;
    float _next_waypoint;
};

} // namespace A320