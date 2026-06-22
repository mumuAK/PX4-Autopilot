/**
 * @file a320_ground_control.cpp
 * @brief A320 地面控制器实现
 */

#include "a320_ground_control.h"

namespace A320 {

GroundController::GroundController()
    : _target_heading(0.0f), _runway_heading(0.0f),
      _nws_cmd(0.0f), _rudder_cmd(0.0f),
      _left_brake_cmd(0.0f), _right_brake_cmd(0.0f),
      _control_mode(GroundControlMode::NWS_ONLY),
      _heading_integral(0.0f), _yaw_rate_integral(0.0f),
      _last_heading_error(0.0f)
{
    // A320 地面控制增益参数
    _gains = {
        // 前轮转弯增益（低速时响应快）
        2.0f, 0.1f, 0.5f,

        // 方向舵增益（高速时响应较慢）
        1.5f, 0.05f, 0.3f,

        // 差动刹车增益
        5.0f, 0.2f,

        // 速度阈值（knots）
        30.0f,   // speed_brake_only - 低于此速度主要用刹车
        80.0f,   // speed_nws_only - 高于此速度主要用方向舵
        130.0f   // speed_rudder_only - 离地速度
    };
}

void GroundController::initialize() {
    _heading_integral = 0.0f;
    _yaw_rate_integral = 0.0f;
    _last_heading_error = 0.0f;
    _nws_cmd = 0.0f;
    _rudder_cmd = 0.0f;
    _left_brake_cmd = 0.0f;
    _right_brake_cmd = 0.0f;
}

void GroundController::setTargetHeading(float heading_deg) {
    _target_heading = Utils::wrapAngle(heading_deg);
}

void GroundController::setRunwayHeading(float runway_heading_deg) {
    _runway_heading = runway_heading_deg * DEG_TO_RAD;
}

void GroundController::update(const AircraftState& state, float dt) {
    // 确定控制模式
    determineControlMode(state);

    // 计算航向误差
    float heading_error = computeHeadingError(state);

    // 根据控制模式计算指令
    switch (_control_mode) {
        case GroundControlMode::DIFFERENTIAL_BRAKE:
            // 极低速：主要使用差动刹车
            computeDifferentialBrake(heading_error, dt);
            _nws_cmd = computeNWS(state, heading_error, dt) * 0.3f;  // 辅助前轮转弯
            _rudder_cmd = 0.0f;
            break;

        case GroundControlMode::NWS_ONLY:
            // 低速：主要使用前轮转弯
            _nws_cmd = computeNWS(state, heading_error, dt);
            _rudder_cmd = computeRudder(state, heading_error, dt) * 0.2f;  // 辅助方向舵
            _left_brake_cmd = 0.0f;
            _right_brake_cmd = 0.0f;
            break;

        case GroundControlMode::NWS_RUDDER:
            // 中速：混合使用前轮和方向舵
            _nws_cmd = computeNWS(state, heading_error, dt);
            _rudder_cmd = computeRudder(state, heading_error, dt);
            blendControls(state.groundspeed * MS_TO_KNOTS);
            break;

        case GroundControlMode::RUDDER_ONLY:
            // 高速/空中：仅使用方向舵
            _nws_cmd = 0.0f;  // 前轮转弯在高速时断开
            _rudder_cmd = computeRudder(state, heading_error, dt);
            _left_brake_cmd = 0.0f;
            _right_brake_cmd = 0.0f;
            break;
    }

    // 应用限制
    float speed_knots = state.groundspeed * MS_TO_KNOTS;
    _nws_cmd = limitNWS(_nws_cmd, speed_knots);
    _rudder_cmd = limitRudder(_rudder_cmd);
}

void GroundController::determineControlMode(const AircraftState& state) {
    float speed_knots = state.groundspeed * MS_TO_KNOTS;

    if (state.is_in_air) {
        // 空中：仅方向舵
        _control_mode = GroundControlMode::RUDDER_ONLY;
    } else if (speed_knots < _gains.speed_brake_only) {
        // 极低速（< 30 kts）：差动刹车
        _control_mode = GroundControlMode::DIFFERENTIAL_BRAKE;
    } else if (speed_knots < _gains.speed_nws_only) {
        // 低速（30-80 kts）：前轮转弯为主
        _control_mode = GroundControlMode::NWS_ONLY;
    } else if (speed_knots < _gains.speed_rudder_only) {
        // 中速（80-130 kts）：混合控制
        _control_mode = GroundControlMode::NWS_RUDDER;
    } else {
        // 高速（> 130 kts）：仅方向舵
        _control_mode = GroundControlMode::RUDDER_ONLY;
    }
}

float GroundController::computeHeadingError(const AircraftState& state) {
    float current_heading = state.yaw * RAD_TO_DEG;
    float error = Utils::wrapAngle(_target_heading - current_heading);

    // 如果有跑道航向信息，优先使用跑道航向误差
    if (state.runway_heading != 0.0f) {
        float runway_heading_deg = state.runway_heading * RAD_TO_DEG;
        float runway_error = Utils::wrapAngle(runway_heading_deg - current_heading);
        // 如果偏离跑道中心线，增加修正力度
        if (std::abs(state.cross_track_error) > 5.0f) {
            error = runway_error + state.cross_track_error * 0.5f;  // 偏航修正
        }
    }

    return error;
}

float GroundController::computeNWS(const AircraftState& state, float heading_error, float dt) {
    // PID 控制
    float proportional = _gains.nws_kp * heading_error;

    _heading_integral += heading_error * dt;
    _heading_integral = Utils::constrain(_heading_integral, -50.0f, 50.0f);
    float integral = _gains.nws_ki * _heading_integral;

    float derivative = 0.0f;
    if (dt > 0.001f) {
        derivative = _gains.nws_kd * (heading_error - _last_heading_error) / dt;
    }
    _last_heading_error = heading_error;

    // 加入偏航速率反馈（阻尼）
    float yaw_rate_feedback = -state.yaw_rate * 2.0f;

    float nws_cmd = (proportional + integral + derivative + yaw_rate_feedback) * DEG_TO_RAD;

    return nws_cmd;
}

float GroundController::computeRudder(const AircraftState& state, float heading_error, float dt) {
    // 方向舵控制需要考虑气动效率随速度变化
    float speed_knots = state.groundspeed * MS_TO_KNOTS;

    // 速度增益调度：低速时方向舵效率低，需要更大的偏角
    float speed_gain = 1.0f;
    if (speed_knots < 60.0f) {
        speed_gain = Utils::interpolate(speed_knots, 0.0f, 60.0f, 3.0f, 1.0f);
    }

    // PID 控制
    float proportional = _gains.rudder_kp * heading_error * speed_gain;

    _yaw_rate_integral += heading_error * dt;
    _yaw_rate_integral = Utils::constrain(_yaw_rate_integral, -30.0f, 30.0f);
    float integral = _gains.rudder_ki * _yaw_rate_integral;

    float derivative = 0.0f;
    if (dt > 0.001f) {
        derivative = _gains.rudder_kd * (heading_error - _last_heading_error) / dt;
    }

    // 偏航速率反馈
    float yaw_rate_feedback = -state.yaw_rate * 1.5f * DEG_TO_RAD;

    float rudder_cmd = (proportional + integral + derivative) * DEG_TO_RAD + yaw_rate_feedback;

    return rudder_cmd;
}

void GroundController::computeDifferentialBrake(float heading_error, float dt) {
    // 差动刹车：左偏航时右刹车，右偏航时左刹车
    float brake_cmd = _gains.brake_kp * std::abs(heading_error);

    _heading_integral += heading_error * dt;
    _heading_integral = Utils::constrain(_heading_integral, -20.0f, 20.0f);
    brake_cmd += _gains.brake_ki * std::abs(_heading_integral);

    brake_cmd = Utils::constrain(brake_cmd, 0.0f, 50.0f);  // 最大50%刹车

    if (heading_error > 0.0f) {
        // 需要右转：左刹车
        _left_brake_cmd = brake_cmd;
        _right_brake_cmd = 0.0f;
    } else {
        // 需要左转：右刹车
        _left_brake_cmd = 0.0f;
        _right_brake_cmd = brake_cmd;
    }
}

void GroundController::blendControls(float speed_knots) {
    // 在 80-130 kts 区间平滑过渡
    if (speed_knots >= 80.0f && speed_knots <= 130.0f) {
        float t = (speed_knots - 80.0f) / 50.0f;

        // 前轮转弯权重逐渐减小
        float nws_weight = 1.0f - t;
        _nws_cmd *= nws_weight;

        // 方向舵权重逐渐增大
        float rudder_weight = t;
        _rudder_cmd *= rudder_weight + 0.3f;  // 保证一定的方向舵输出
    }
}

float GroundController::limitNWS(float nws, float speed_knots) {
    // A320 前轮转弯限制
    float max_angle_deg;

    if (speed_knots < 10.0f) {
        // 极低速：最大 ±75°（拖飞机时）
        max_angle_deg = 75.0f;
    } else if (speed_knots < 80.0f) {
        // 低速：最大 ±30°（正常滑行）
        max_angle_deg = Utils::interpolate(speed_knots, 10.0f, 80.0f, 30.0f, 6.0f);
    } else {
        // 高速：最大 ±6°（起飞滑跑）
        max_angle_deg = 6.0f;
    }

    return Utils::constrain(nws, -max_angle_deg * DEG_TO_RAD, max_angle_deg * DEG_TO_RAD);
}

float GroundController::limitRudder(float rudder) {
    // A320 方向舵限制：±25°
    return Utils::constrain(rudder, -25.0f * DEG_TO_RAD, 25.0f * DEG_TO_RAD);
}

const char* GroundController::getControlModeName() const {
    switch (_control_mode) {
        case GroundControlMode::DIFFERENTIAL_BRAKE: return "Differential Brake";
        case GroundControlMode::NWS_ONLY: return "NWS Only";
        case GroundControlMode::NWS_RUDDER: return "NWS + Rudder";
        case GroundControlMode::RUDDER_ONLY: return "Rudder Only";
        default: return "Unknown";
    }
}

} // namespace A320