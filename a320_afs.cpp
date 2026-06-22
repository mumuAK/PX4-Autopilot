/**
 * @file a320_afs.cpp
 * @brief A320 自动飞行系统实现
 */

#include "a320_afs.h"

namespace A320 {

AutoFlightSystem::AutoFlightSystem()
    : _ap_enabled(false), _at_enabled(false)
{
}

void AutoFlightSystem::initialize() {
    _fcc.initialize();
    _autothrottle.setMode(false);
    _fms.initialize();
}

void AutoFlightSystem::enableAutopilot() {
    _ap_enabled = true;
}

void AutoFlightSystem::disableAutopilot() {
    _ap_enabled = false;
    _fcc.setAFMode(AFMode::OFF);
}

void AutoFlightSystem::enableAutothrottle() {
    _at_enabled = true;
    _autothrottle.setMode(true);
}

void AutoFlightSystem::disableAutothrottle() {
    _at_enabled = false;
    _autothrottle.setMode(false);
}

void AutoFlightSystem::setAFMode(AFMode mode) {
    if (_ap_enabled) {
        _fcc.setAFMode(mode);
    }
}

void AutoFlightSystem::setTargetAltitude(float altitude_ft) {
    _fcc.setTargetAltitude(altitude_ft);
    _fms.setTargetAltitude(altitude_ft);
}

void AutoFlightSystem::setTargetVerticalSpeed(float vs_fpm) {
    _fcc.setTargetVerticalSpeed(vs_fpm);
    _fms.setTargetVS(vs_fpm);
}

void AutoFlightSystem::setTargetSpeed(float speed_knots) {
    _fcc.setTargetSpeed(speed_knots);
    _fms.setTargetSpeed(speed_knots);
    _autothrottle.setTargetSpeed(speed_knots);
}

void AutoFlightSystem::setTargetMach(float mach) {
    _fcc.setTargetMach(mach);
    _fms.setTargetMach(mach);
    _autothrottle.setTargetMach(mach);
}

void AutoFlightSystem::setTargetHeading(float heading_deg) {
    _fcc.setTargetHeading(heading_deg);
    _fms.setTargetHeading(heading_deg);
}

void AutoFlightSystem::setRunwayHeading(float runway_heading_deg) {
    _fcc.setRunwayHeading(runway_heading_deg);
}

void AutoFlightSystem::activateGoAround(const AircraftState& state) {
    _fcc.activateGoAround(state);
    _autothrottle.setFlightPhase(FlightPhase::GO_AROUND);
    _autothrottle.setTargetSpeed(_fcc.getGoAroundController().getTargetSpeed());
}

void AutoFlightSystem::activateEngineOut(const AircraftState& state, const std::string& failed_side) {
    _fcc.activateEngineOut(state, failed_side);
    _autothrottle.setFlightPhase(FlightPhase::CLIMB);
    _autothrottle.setTargetThrust(_fcc.getEngineOutController().getTargetThrust());
}

void AutoFlightSystem::update(const AircraftState& state) {
    _control_command = {};

    if (_ap_enabled) {
        handleModeTransitions(state);

        ControlCommand cmd = _fcc.computeControl(state);
        _control_command = cmd;
    }

    if (_at_enabled) {
        _control_command.throttle_cmd = _autothrottle.computeThrottle(state);
    }

    enforceFlightEnvelope(state);
}

void AutoFlightSystem::handleModeTransitions(const AircraftState& state) {
    AFMode current_mode = _fcc.getAFMode();

    if (current_mode == AFMode::GO_AROUND) {
        if (_fcc.getGoAroundController().isCompleted()) {
            setAFMode(AFMode::VERTICAL_SPEED);
            setTargetVerticalSpeed(_fcc.getGoAroundController().getTargetVerticalSpeed());
            setTargetSpeed(_fcc.getGoAroundController().getTargetSpeed());
        }
    }

    // 单发失效稳定后可以切换到正常爬升
    if (current_mode == AFMode::ENGINE_OUT) {
        if (_fcc.getEngineOutController().isStabilized()) {
            // 保持单发模式，但可以调整目标参数
        }
    }
}

void AutoFlightSystem::enforceFlightEnvelope(const AircraftState& state) {
    if (state.aoa > state.alpha_prot * 0.85f) {
        float aoa_margin = state.alpha_prot - state.aoa;
        float pitch_reduction = std::max(0.0f, -aoa_margin * RAD_TO_DEG * 2.0f) * DEG_TO_RAD;
        _control_command.pitch_cmd = Utils::constrain(
            _control_command.pitch_cmd - pitch_reduction,
            -15.0f * DEG_TO_RAD, 10.0f * DEG_TO_RAD);

        _control_command.throttle_cmd = Utils::constrain(
            _control_command.throttle_cmd - 3.0f, 0.0f, 100.0f);
    }

    if (state.mach > 0.86f) {
        _control_command.throttle_cmd = Utils::constrain(
            _control_command.throttle_cmd - 10.0f, 0.0f, 100.0f);
    }

    float speed_knots = state.cas * MS_TO_KNOTS;
    if (speed_knots < 140.0f && _fcc.getAFMode() != AFMode::GO_AROUND) {
        _control_command.throttle_cmd = Utils::constrain(
            _control_command.throttle_cmd + 5.0f, 0.0f, 100.0f);
    }
}

float AutoFlightSystem::getGoAroundTargetSpeed() const {
    return _fcc.getGoAroundController().getTargetSpeed();
}

float AutoFlightSystem::getGoAroundTargetVS() const {
    return _fcc.getGoAroundController().getTargetVerticalSpeed();
}

const char* AutoFlightSystem::getGoAroundPhaseName() const {
    return _fcc.getGoAroundController().getPhaseName();
}

bool AutoFlightSystem::isGoAroundCompleted() const {
    return _fcc.getGoAroundController().isCompleted();
}

} // namespace A320