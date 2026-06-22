/**
 * @file a320_engine_out.cpp
 * @brief A320 单发失效补偿控制器实现
 */

#include "a320_engine_out.h"

namespace A320 {

EngineOutController::EngineOutController()
    : _is_active(false), _left_engine_failed(false), _right_engine_failed(false),
      _time_since_failure(0.0f), _phase(EngineFailurePhase::NONE),
      _rudder_trim(0.0f), _aileron_trim(0.0f), _rudder_cmd(0.0f),
      _target_speed(200.0f), _target_thrust(85.0f),
      _yaw_integral(0.0f), _roll_integral(0.0f),
      _last_yaw_error(0.0f), _last_asymmetry(0.0f)
{
    // A320 单发失效控制增益参数
    _gains = {
        // 方向舵增益（单发时需要更强的控制）
        3.0f, 0.2f, 0.8f,

        // 副翼增益
        1.5f, 0.05f,

        // 推力不对称补偿增益
        // A320 发动机横向距离约 5.5m，推力差约 50000N
        // 需要约 5-10° 方向舵配平
        0.0001f,  // asymmetry_kp: 每牛顿推力差需要的方向舵偏角
        0.00001f, // asymmetry_ki

        // 配平速率限制：A320 约 1 deg/s
        1.0f
    };
}

void EngineOutController::initialize() {
    reset();
}

void EngineOutController::activate(const AircraftState& state, const std::string& failed_side) {
    _is_active = true;
    _time_since_failure = 0.0f;
    _phase = EngineFailurePhase::IMMEDIATE;

    if (failed_side == "LEFT") {
        _left_engine_failed = true;
        _right_engine_failed = false;
    } else {
        _left_engine_failed = false;
        _right_engine_failed = true;
    }

    // 初始化目标速度和推力
    _target_speed = computeTargetSpeed(state);
    _target_thrust = computeTargetThrust(state);

    // 重置积分器
    _yaw_integral = 0.0f;
    _roll_integral = 0.0f;
    _last_yaw_error = 0.0f;
    _last_asymmetry = 0.0f;

    // 初始方向舵指令（立即响应）
    _rudder_cmd = computeImmediateRudder(state);
}

void EngineOutController::reset() {
    _is_active = false;
    _left_engine_failed = false;
    _right_engine_failed = false;
    _time_since_failure = 0.0f;
    _phase = EngineFailurePhase::NONE;
    _rudder_trim = 0.0f;
    _aileron_trim = 0.0f;
    _rudder_cmd = 0.0f;
    _yaw_integral = 0.0f;
    _roll_integral = 0.0f;
}

void EngineOutController::update(const AircraftState& state, float dt) {
    if (!_is_active) return;

    _time_since_failure += dt;

    // 更新失效阶段
    updatePhase(state);

    // 计算补偿指令
    float asymmetry_comp = computeAsymmetryCompensation(state, dt);
    float yaw_comp = computeYawCompensation(state, dt);
    float roll_comp = computeRollCompensation(state, dt);

    // 根据阶段调整控制策略
    switch (_phase) {
        case EngineFailurePhase::IMMEDIATE:
            // 立即响应：快速踩舵，配平逐渐建立
            _rudder_cmd = computeImmediateRudder(state);
            _rudder_trim = limitTrimRate(_rudder_trim, asymmetry_comp * 0.5f, dt);
            _aileron_trim = limitTrimRate(_aileron_trim, roll_comp * 0.3f, dt);
            break;

        case EngineFailurePhase::STABILIZATION:
            // 稳定阶段：建立配平，减少脚蹬输入
            _rudder_cmd = yaw_comp * 0.5f;  // 减少脚蹬输入
            _rudder_trim = limitTrimRate(_rudder_trim, asymmetry_comp, dt);
            _aileron_trim = limitTrimRate(_aileron_trim, roll_comp, dt);
            break;

        case EngineFailurePhase::CLIMB_OUT:
        case EngineFailurePhase::CRUISE:
            // 爬升/巡航：主要使用配平
            _rudder_cmd = yaw_comp * 0.2f;  // 仅少量脚蹬修正
            _rudder_trim = limitTrimRate(_rudder_trim, asymmetry_comp, dt);
            _aileron_trim = limitTrimRate(_aileron_trim, roll_comp * 0.5f, dt);
            break;

        case EngineFailurePhase::NONE:
            break;
    }

    // 更新目标速度和推力
    _target_speed = computeTargetSpeed(state);
    _target_thrust = computeTargetThrust(state);
}

void EngineOutController::updatePhase(const AircraftState& state) {
    switch (_phase) {
        case EngineFailurePhase::NONE:
            break;

        case EngineFailurePhase::IMMEDIATE:
            // 0-5秒：立即响应阶段
            if (_time_since_failure > 5.0f) {
                _phase = EngineFailurePhase::STABILIZATION;
            }
            break;

        case EngineFailurePhase::STABILIZATION: {
            // 5-30秒：稳定阶段
            // 检查是否已稳定（航向和滚转误差小）
            float heading_error = std::abs(state.yaw - state.ground_track);
            float roll_error = std::abs(state.roll);

            if (_time_since_failure > 30.0f ||
                (heading_error < 3.0f * DEG_TO_RAD && roll_error < 5.0f * DEG_TO_RAD)) {
                _phase = EngineFailurePhase::CLIMB_OUT;
            }
            break;
        }

        case EngineFailurePhase::CLIMB_OUT:
            // 爬升阶段：到达安全高度后进入巡航
            if (state.alt_std > 1500.0f * 0.3048f) {  // 1500 ft
                _phase = EngineFailurePhase::CRUISE;
            }
            break;

        case EngineFailurePhase::CRUISE:
            // 巡航阶段：保持
            break;
    }
}

float EngineOutController::computeAsymmetryCompensation(const AircraftState& state, float dt) {
    // 计算推力不对称矩
    float asymmetry_moment = Utils::computeAsymmetricMoment(
        state.thrust_left, state.thrust_right);

    // 计算需要的方向舵偏角
    // 方向舵产生的侧力矩 = 方向舵偏角 × 速度 × 效率系数
    float speed_knots = state.cas * MS_TO_KNOTS;
    float rudder_effectiveness = Utils::interpolate(speed_knots, 100.0f, 250.0f, 0.5f, 1.0f);

    // 每度方向舵产生的侧力矩（简化模型）
    float rudder_moment_per_deg = 1000.0f * rudder_effectiveness;  // N·m/deg

    float required_rudder_deg = asymmetry_moment / rudder_moment_per_deg;

    // PID 控制
    float proportional = _gains.asymmetry_kp * asymmetry_moment;

    _last_asymmetry += asymmetry_moment * dt;
    _last_asymmetry = Utils::constrain(_last_asymmetry, -100000.0f, 100000.0f);
    float integral = _gains.asymmetry_ki * _last_asymmetry;

    float rudder_trim = (proportional + integral) * RAD_TO_DEG;  // 转换为度

    // 限制配平范围：A320 方向舵配平最大约 15°
    rudder_trim = Utils::constrain(rudder_trim, -15.0f, 15.0f);

    return rudder_trim * DEG_TO_RAD;
}

float EngineOutController::computeYawCompensation(const AircraftState& state, float dt) {
    // 航向误差（相对于目标航向或跑道航向）
    float target_heading = state.ground_track;  // 使用地面轨迹作为目标
    float heading_error = Utils::wrapAngleRad(target_heading - state.yaw);

    // PID 控制
    float proportional = _gains.rudder_kp * heading_error * RAD_TO_DEG;

    _yaw_integral += heading_error * dt;
    _yaw_integral = Utils::constrain(_yaw_integral, -10.0f, 10.0f);
    float integral = _gains.rudder_ki * _yaw_integral;

    float derivative = 0.0f;
    if (dt > 0.001f) {
        derivative = _gains.rudder_kd * (heading_error - _last_yaw_error) / dt;
    }
    _last_yaw_error = heading_error;

    // 偏航速率反馈（阻尼）
    float yaw_rate_feedback = -state.yaw_rate * 2.0f;

    float rudder_cmd = (proportional + integral + derivative + yaw_rate_feedback) * DEG_TO_RAD;

    // 限制方向舵范围
    return Utils::constrain(rudder_cmd, -25.0f * DEG_TO_RAD, 25.0f * DEG_TO_RAD);
}

float EngineOutController::computeRollCompensation(const AircraftState& state, float dt) {
    // 单发失效时，失效侧发动机推力小，飞机倾向于向失效侧滚转
    // 需要向工作侧配平副翼

    float target_roll = 0.0f;  // 目标是保持机翼水平
    float roll_error = target_roll - state.roll;

    // PID 控制
    float proportional = _gains.aileron_kp * roll_error * RAD_TO_DEG;

    _roll_integral += roll_error * dt;
    _roll_integral = Utils::constrain(_roll_integral, -5.0f, 5.0f);
    float integral = _gains.aileron_ki * _roll_integral;

    float aileron_trim = (proportional + integral) * DEG_TO_RAD;

    // 限制副翼配平范围
    return Utils::constrain(aileron_trim, -10.0f * DEG_TO_RAD, 10.0f * DEG_TO_RAD);
}

float EngineOutController::computeTargetSpeed(const AircraftState& state) {
    // 单发爬升速度：V2 + 10 到 V2 + 20
    // A320 单发 V2 约 145-160 knots

    float current_speed = state.cas * MS_TO_KNOTS;

    // 根据阶段调整目标速度
    float target_speed;
    switch (_phase) {
        case EngineFailurePhase::IMMEDIATE:
            // 立即响应：保持当前速度
            target_speed = current_speed;
            break;

        case EngineFailurePhase::STABILIZATION:
            // 稳定阶段：调整到单发爬升速度
            target_speed = std::max(current_speed, 160.0f);  // V2 + 10
            break;

        case EngineFailurePhase::CLIMB_OUT:
            // 爬升阶段：保持单发爬升速度
            target_speed = 170.0f;  // V2 + 20
            break;

        case EngineFailurePhase::CRUISE:
            // 巡航阶段：单发巡航速度
            target_speed = 200.0f;  // M0.60 左右
            break;

        default:
            target_speed = current_speed;
    }

    return target_speed;
}

float EngineOutController::computeTargetThrust(const AircraftState& state) {
    // 单发时，工作发动机需要增加到最大连续推力（MCT）
    // A320 MCT 约 85-90% N1

    float target_thrust;

    switch (_phase) {
        case EngineFailurePhase::IMMEDIATE:
            // 立即响应：保持或增加推力
            target_thrust = std::max(state.n1_left, state.n1_right);
            target_thrust = std::min(target_thrust + 10.0f, 95.0f);
            break;

        case EngineFailurePhase::STABILIZATION:
        case EngineFailurePhase::CLIMB_OUT:
            // 爬升阶段：MCT
            target_thrust = 90.0f;
            break;

        case EngineFailurePhase::CRUISE:
            // 巡航阶段：根据需要调整
            target_thrust = 85.0f;
            break;

        default:
            target_thrust = 80.0f;
    }

    return target_thrust;
}

float EngineOutController::limitTrimRate(float current, float target, float dt) {
    float max_rate = _gains.trim_rate_max * DEG_TO_RAD;  // deg/s → rad/s
    return Utils::limitRate(current, target, max_rate, dt);
}

float EngineOutController::computeImmediateRudder(const AircraftState& state) {
    // 在失效瞬间，快速踩舵抵消偏航趋势
    // 左发失效 → 向左偏航 → 需要右舵
    // 右发失效 → 向右偏航 → 需要左舵

    float rudder_cmd = 0.0f;

    if (_left_engine_failed) {
        // 左发失效，飞机向左偏航，需要右舵（正值）
        // 根据推力差计算需要的方向舵偏角
        float thrust_diff = state.thrust_right - state.thrust_left;
        float speed_knots = state.cas * MS_TO_KNOTS;

        // 简化模型：每 10000N 推力差需要约 2° 方向舵（低速）
        float rudder_per_10kN = Utils::interpolate(speed_knots, 100.0f, 200.0f, 3.0f, 1.5f);
        rudder_cmd = thrust_diff / 10000.0f * rudder_per_10kN * DEG_TO_RAD;

    } else if (_right_engine_failed) {
        // 右发失效，飞机向右偏航，需要左舵（负值）
        float thrust_diff = state.thrust_right - state.thrust_left;
        float speed_knots = state.cas * MS_TO_KNOTS;

        float rudder_per_10kN = Utils::interpolate(speed_knots, 100.0f, 200.0f, 3.0f, 1.5f);
        rudder_cmd = thrust_diff / 10000.0f * rudder_per_10kN * DEG_TO_RAD;
    }

    // 立即响应阶段使用更大的方向舵偏角
    rudder_cmd *= 1.5f;

    return Utils::constrain(rudder_cmd, -20.0f * DEG_TO_RAD, 20.0f * DEG_TO_RAD);
}

const char* EngineOutController::getPhaseName() const {
    switch (_phase) {
        case EngineFailurePhase::NONE: return "None";
        case EngineFailurePhase::IMMEDIATE: return "Immediate Response";
        case EngineFailurePhase::STABILIZATION: return "Stabilization";
        case EngineFailurePhase::CLIMB_OUT: return "Climb Out";
        case EngineFailurePhase::CRUISE: return "Cruise";
        default: return "Unknown";
    }
}

} // namespace A320