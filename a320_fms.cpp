#include "a320_fms.h"

namespace A320 {

FlightManagementSystem::FlightManagementSystem()
    : _target_altitude(35000.0f), _target_speed(250.0f),
      _target_mach(0.78f), _target_heading(0.0f),
      _target_vs(1500.0f), _current_waypoint(0), _next_waypoint(1)
{
}

void FlightManagementSystem::initialize() {
    _target_altitude = 35000.0f;
    _target_speed = 250.0f;
    _target_mach = 0.78f;
    _target_heading = 0.0f;
    _target_vs = 1500.0f;
}

void FlightManagementSystem::setFlightPlan() {
}

void FlightManagementSystem::update(const AircraftState& state) {
}

} // namespace A320