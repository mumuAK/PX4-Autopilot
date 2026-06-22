/**
 * @file test_driver.cpp
 * @brief A320 自动驾驶系统主控类实现
 */

#include "test_driver.h"
#include <string>

namespace A320 {

test_driver::test_driver() {
}

void test_driver::init() {
    _afs.initialize();
}

void test_driver::map_in(const AircraftState& state) {
    _current_state = state;
}

void test_driver::step(float dt) {
    (void)dt;
    _afs.update(_current_state);
    _current_command = _afs.getControlCommand();
}

void test_driver::map_out(ControlCommand& cmd) {
    cmd = _current_command;
}

void test_driver::execute_command(const std::string& command, float value) {
    if (command == "AP_ON") {
        _afs.enableAutopilot();
    } else if (command == "AP_OFF") {
        _afs.disableAutopilot();
    } else if (command == "AT_ON") {
        _afs.enableAutothrottle();
    } else if (command == "AT_OFF") {
        _afs.disableAutothrottle();
    } else if (command == "HDG_HOLD") {
        _afs.setAFMode(AFMode::HEADING_HOLD);
    } else if (command == "ALT_HOLD") {
        _afs.setAFMode(AFMode::ALTITUDE_HOLD);
    } else if (command == "VS_HOLD") {
        _afs.setAFMode(AFMode::VERTICAL_SPEED);
    } else if (command == "SPD_HOLD") {
        _afs.setAFMode(AFMode::SPEED_HOLD);
    } else if (command == "ATT_HOLD") {
        _afs.setAFMode(AFMode::ATTITUDE_HOLD);
    } else if (command == "GO_AROUND") {
        _afs.activateGoAround(_current_state);
    } else if (command == "CLIMB") {
        _afs.setAFMode(AFMode::CLIMB);
    } else if (command == "DESCENT") {
        _afs.setAFMode(AFMode::DESCENT);
    } else if (command == "GROUND_TRACK") {
        _afs.setAFMode(AFMode::GROUND_TRACK);
    } else if (command == "ENGINE_OUT_LEFT") {
        _afs.activateEngineOut(_current_state, "LEFT");
    } else if (command == "ENGINE_OUT_RIGHT") {
        _afs.activateEngineOut(_current_state, "RIGHT");
    } else if (command == "SET_HDG") {
        _afs.setTargetHeading(value);
    } else if (command == "SET_ALT") {
        _afs.setTargetAltitude(value);
    } else if (command == "SET_VS") {
        _afs.setTargetVerticalSpeed(value);
    } else if (command == "SET_SPD") {
        _afs.setTargetSpeed(value);
    } else if (command == "SET_MACH") {
        _afs.setTargetMach(value);
    } else if (command == "SET_RWY_HDG") {
        _afs.setRunwayHeading(value);
    }
}

} // namespace A320