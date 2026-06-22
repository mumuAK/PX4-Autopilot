/**
 * @file a320_afs.h
 * @brief A320 自动飞行系统（AutoFlightSystem）
 */

#pragma once

#include "a320_autopilot_common.h"
#include "a320_fcc.h"
#include "a320_autothrottle.h"
#include "a320_fms.h"

namespace A320 {

class AutoFlightSystem {
public:
    AutoFlightSystem();

    void initialize();

    void enableAutopilot();
    void disableAutopilot();
    bool isAutopilotEnabled() const { return _ap_enabled; }

    void enableAutothrottle();
    void disableAutothrottle();
    bool isAutothrottleEnabled() const { return _at_enabled; }

    void setAFMode(AFMode mode);
    AFMode getAFMode() const { return _fcc.getAFMode(); }

    void setTargetAltitude(float altitude_ft);
    void setTargetVerticalSpeed(float vs_fpm);
    void setTargetSpeed(float speed_knots);
    void setTargetMach(float mach);
    void setTargetHeading(float heading_deg);
    void setRunwayHeading(float runway_heading_deg);

    void activateGoAround(const AircraftState& state);
    void activateEngineOut(const AircraftState& state, const std::string& failed_side);

    bool isGoAroundActive() const { return _fcc.isGoAroundActive(); }
    bool isEngineOutActive() const { return _fcc.isEngineOutActive(); }

    const GoAroundController& getGoAroundController() const { return _fcc.getGoAroundController(); }
    const GroundController& getGroundController() const { return _fcc.getGroundController(); }
    const EngineOutController& getEngineOutController() const { return _fcc.getEngineOutController(); }

    void update(const AircraftState& state);

    ControlCommand getControlCommand() const { return _control_command; }

    float getGoAroundTargetSpeed() const;
    float getGoAroundTargetVS() const;
    const char* getGoAroundPhaseName() const;
    bool isGoAroundCompleted() const;

private:
    bool _ap_enabled;
    bool _at_enabled;

    FlightControlComputer _fcc;
    Autothrottle _autothrottle;
    FlightManagementSystem _fms;

    ControlCommand _control_command;

    void handleModeTransitions(const AircraftState& state);
    void enforceFlightEnvelope(const AircraftState& state);
};

} // namespace A320