#pragma once

#include "a320_autopilot_common.h"
#include "a320_afs.h"

namespace A320 {

class test_driver {
public:
    test_driver();

    void init();

    void map_in(const AircraftState& state);

    void step(float dt = 0.05f);

    void map_out(ControlCommand& cmd);

    void execute_command(const std::string& command, float value = 0.0f);

    bool is_autopilot_enabled() const { return _afs.isAutopilotEnabled(); }
    bool is_autothrottle_enabled() const { return _afs.isAutothrottleEnabled(); }
    AFMode get_current_mode() const { return _afs.getAFMode(); }

private:
    AutoFlightSystem _afs;
    AircraftState _current_state;
    ControlCommand _current_command;
};

} // namespace A320