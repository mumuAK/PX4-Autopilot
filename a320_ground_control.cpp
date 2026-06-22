/**
 * @file a320_ground_control.cpp
 * @brief A320 地面控制器实现（简化版）
 */

#include "a320_ground_control.h"

namespace A320 {

// 速度阈值
constexpr float SPEED_NWS_THRESHOLD = 30.0f;  // kts

GroundController::GroundController()
    : _target_heading(0.0f), _target_speed(0.0f), _runway_heading(0.0f),
      _nws_cmd(0.0f), _rudder_cmd(0.0f), _brake_cmd(0.0f), _heading_integral(0.0f)
{
}

void GroundController::initialize() {
    _nws_cmd = 0.0f;
    _rudder_cmd = 0.0f;
    _brake_cmd = 0.0f;
    _heading_integral = 0.0f;
}

void GroundController::setTargetHeading(float heading_deg) {
    _target_heading = Utils::wrapAngle(heading_deg);
}

void GroundController::setTargetSpeed(float speed_knots) {
    _target_speed = speed_knots;
}

void GroundController::setRunwayHeading(float runway_heading_deg) {
    _runway_heading = Utils::wrapAngle(runway_heading_deg) * DEG_TO_RAD;
}

void GroundController::update(const AircraftState& state, float dt) {
    float speed_knots = state.groundspeed * MS_TO_KNOTS;
    float heading_error = computeHeadingError(state);

    // PID 控制：P + I
    float proportional = heading_error;
    _heading_integral += heading_error * dt;
    _heading_integral = Utils::constrain(_heading_integral, -50.0f, 50.0f);
    float integral = _heading_integral * 0.1f;

    float control_output = proportional + integral;

    // 根据速度选择控制方式
    if (speed_knots < SPEED_NWS_THRESHOLD) {
        // 低速：使用前轮转弯
        // 航向误差 → 前轮转角
        _nws_cmd = control_output * 1.5f;  // 放大系数
        _nws_cmd = limitNWS(_nws_cmd, speed_knots);
        _rudder_cmd = 0.0f;
    } else {
        // 高速：使用方向舵
        // 航向误差 → 方向舵偏角
        _rudder_cmd = control_output * 0.5f * DEG_TO_RAD;
        _rudder_cmd = Utils::constrain(_rudder_cmd, -25.0f * DEG_TO_RAD, 25.0f * DEG_TO_RAD);
        _nws_cmd = 0.0f;
    }

    // 速度保持：使用刹车
    float speed_error = _target_speed - speed_knots;
    if (std::abs(speed_error) > 5.0f) {
        _brake_cmd = Utils::constrain(speed_error * 2.0f, 0.0f, 100.0f);
    } else {
        _brake_cmd = 0.0f;
    }
}

float GroundController::computeHeadingError(const AircraftState& state) {
    float current_heading = state.yaw * RAD_TO_DEG;
    return Utils::wrapAngle(_target_heading - current_heading);
}

float GroundController::limitNWS(float nws, float speed_knots) {
    // 前轮转弯限制随速度变化
    float max_angle_deg;
    if (speed_knots < 10.0f) {
        max_angle_deg = 75.0f;  // 拖飞机
    } else if (speed_knots < 30.0f) {
        max_angle_deg = Utils::interpolate(speed_knots, 10.0f, 30.0f, 30.0f, 6.0f);
    } else {
        max_angle_deg = 6.0f;
    }
    return Utils::constrain(nws, -max_angle_deg * DEG_TO_RAD, max_angle_deg * DEG_TO_RAD);
}

} // namespace A320