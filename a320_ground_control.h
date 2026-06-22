/**
 * @file a320_ground_control.h
 * @brief A320 地面控制器（简化版）
 * 
 * 简化设计：
 * - 速度 < 30 kts: 使用前轮转弯（NWS）
 * - 速度 >= 30 kts: 使用脚蹬（方向舵）
 * - 速度保持: 使用刹车
 */

#pragma once

#include "a320_autopilot_common.h"

namespace A320 {

/**
 * @class GroundController
 * @brief 地面控制器（简化版）
 */
class GroundController {
public:
    GroundController();

    void initialize();
    void setTargetHeading(float heading_deg);
    void setTargetSpeed(float speed_knots);
    void setRunwayHeading(float runway_heading_deg);

    void update(const AircraftState& state, float dt);

    float getNWSCommand() const { return _nws_cmd; }
    float getRudderCommand() const { return _rudder_cmd; }
    float getBrakeCommand() const { return _brake_cmd; }
    float getLeftBrakeCommand() const { return _brake_cmd; }
    float getRightBrakeCommand() const { return 0.0f; }

private:
    float _target_heading;
    float _target_speed;
    float _runway_heading;

    float _nws_cmd;
    float _rudder_cmd;
    float _brake_cmd;

    float _heading_integral;

    float computeHeadingError(const AircraftState& state);
    float limitNWS(float nws, float speed_knots);
};

} // namespace A320