/**
 * @file a320_engine_out.cpp
 * @brief 单发失效脚蹬补偿控制器实现（简化版）
 */

#include "a320_engine_out.h"

namespace A320 {

EngineOutController::EngineOutController()
    : _is_active(false), _left_engine_failed(false), _right_engine_failed(false),
      _rudder_trim(0.0f), _rudder_cmd(0.0f), _trim_rate_max(0.03f)  // ~2 deg/s
{
}

void EngineOutController::initialize() {
    reset();
}

void EngineOutController::activate(const AircraftState& state, const std::string& failed_side) {
    _is_active = true;

    if (failed_side == "LEFT") {
        _left_engine_failed = true;
        _right_engine_failed = false;
    } else {
        _left_engine_failed = false;
        _right_engine_failed = true;
    }

    // 立即计算初始方向舵补偿
    _rudder_cmd = computeImmediateRudder(state);
    _rudder_trim = _rudder_cmd;  // 配平缓慢建立到相同值
}

void EngineOutController::reset() {
    _is_active = false;
    _left_engine_failed = false;
    _right_engine_failed = false;
    _rudder_trim = 0.0f;
    _rudder_cmd = 0.0f;
}

void EngineOutController::update(const AircraftState& state, float dt) {
    if (!_is_active) return;

    // 计算立即响应方向舵（快速抵消偏航趋势）
    _rudder_cmd = computeImmediateRudder(state);

    // 缓慢建立配平（防止配平变化过快）
    _rudder_trim = Utils::limitRate(_rudder_trim, _rudder_cmd, _trim_rate_max, dt);
}

float EngineOutController::computeImmediateRudder(const AircraftState& state) {
    // 根据推力不对称计算需要的方向舵偏角
    float thrust_diff = state.thrust_right - state.thrust_left;

    // A320 发动机横向距离约 5.5m
    // 简化模型：每 10000N 推力差需要约 2° 方向舵（气动效率随速度变化）
    float speed_knots = state.cas * MS_TO_KNOTS;
    float rudder_per_10kN = Utils::interpolate(speed_knots, 100.0f, 200.0f, 4.0f, 2.0f);

    float rudder_deg = thrust_diff / 10000.0f * rudder_per_10kN;

    // 左发失效 → 飞机向左偏航 → 需要右舵（正值）
    if (_left_engine_failed) {
        // 已经由 thrust_diff 方向确定
    }

    // 限制方向舵范围：A320 方向舵配平约 ±15°
    rudder_deg = Utils::constrain(rudder_deg, -15.0f, 15.0f);

    return rudder_deg * DEG_TO_RAD;
}

float EngineOutController::computeRudderTrim(const AircraftState& state, float dt) {
    // 配平使用积分控制缓慢调整
    (void)state;
    (void)dt;
    return _rudder_cmd;
}

const char* EngineOutController::getPhaseName() const {
    if (!_is_active) return "Normal";
    return _left_engine_failed ? "Engine Out (Left)" : "Engine Out (Right)";
}

} // namespace A320