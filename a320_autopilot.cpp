#include "a320_autopilot.h"

namespace A320 {

// ============================================
// Autothrottle Implementation
// ============================================
Autothrottle::Autothrottle() 
    : _enabled(false), _target_speed(250.0f), _target_mach(0.78f),
      _target_thrust(80.0f), _target_epr(1.0f),
      _flight_phase(FlightPhase::CRUISE), _last_speed_error(0.0f), _last_time(0.0f)
{
    // 初始化PID参数 - A320典型值
    _speed_pid = {0.5f, 0.02f, 0.1f, 0.0f, 10.0f, 0.0f, 100.0f};
    _mach_pid = {0.8f, 0.01f, 0.15f, 0.0f, 5.0f, 0.0f, 100.0f};
    _thrust_pid = {1.0f, 0.05f, 0.2f, 0.0f, 20.0f, 0.0f, 100.0f};
}

void Autothrottle::setMode(bool enabled) {
    _enabled = enabled;
    if (!enabled) {
        _speed_pid.integral = 0.0f;
        _mach_pid.integral = 0.0f;
        _thrust_pid.integral = 0.0f;
    }
}

void Autothrottle::setTargetSpeed(float speed_knots) {
    _target_speed = speed_knots;
}

void Autothrottle::setTargetMach(float mach) {
    _target_mach = mach;
}

void Autothrottle::setTargetThrust(float n1_percent) {
    _target_thrust = n1_percent;
}

void Autothrottle::setTargetEPR(float epr) {
    _target_epr = epr;
}

void Autothrottle::setFlightPhase(FlightPhase phase) {
    _flight_phase = phase;
}

float Autothrottle::computeThrottle(const AircraftState& state) {
    if (!_enabled) {
        return 0.0f;
    }
    
    float throttle = 0.0f;
    
    switch (_flight_phase) {
        case FlightPhase::TAKEOFF:
        case FlightPhase::CLIMB:
            throttle = computeThrustControl(state);
            break;
            
        case FlightPhase::CRUISE:
            if (state.mach < 0.4f) {
                throttle = computeSpeedControl(state);
            } else {
                throttle = computeMachControl(state);
            }
            break;
            
        case FlightPhase::DESCENT:
        case FlightPhase::APPROACH:
            throttle = computeSpeedControl(state);
            break;
            
        case FlightPhase::GO_AROUND:
            throttle = 95.0f; // GA推力
            break;
            
        default:
            throttle = 0.0f;
    }
    
    return Utils::constrain(throttle, 0.0f, 100.0f);
}

float Autothrottle::computeSpeedControl(const AircraftState& state) {
    float current_speed = state.cas * MS_TO_KNOTS;
    float error = _target_speed - current_speed;
    
    float dt = 0.05f; // 假设50Hz更新
    
    _speed_pid.integral += error * dt;
    _speed_pid.integral = Utils::constrain(_speed_pid.integral, 
                                           -_speed_pid.max_integral, 
                                           _speed_pid.max_integral);
    
    float derivative = (error - _last_speed_error) / dt;
    _last_speed_error = error;
    
    float output = _speed_pid.kp * error +
                   _speed_pid.ki * _speed_pid.integral +
                   _speed_pid.kd * derivative;
    
    return Utils::constrain(output + 50.0f, _speed_pid.min_output, _speed_pid.max_output);
}

float Autothrottle::computeMachControl(const AircraftState& state) {
    float error = _target_mach - state.mach;
    
    float dt = 0.05f;
    
    _mach_pid.integral += error * dt;
    _mach_pid.integral = Utils::constrain(_mach_pid.integral,
                                          -_mach_pid.max_integral,
                                          _mach_pid.max_integral);
    
    float output = _mach_pid.kp * error +
                   _mach_pid.ki * _mach_pid.integral;
    
    return Utils::constrain(output + 50.0f, _mach_pid.min_output, _mach_pid.max_output);
}

float Autothrottle::computeThrustControl(const AircraftState& state) {
    float current_thrust = (state.n1_left + state.n1_right) / 2.0f;
    float error = _target_thrust - current_thrust;
    
    float dt = 0.05f;
    
    _thrust_pid.integral += error * dt;
    _thrust_pid.integral = Utils::constrain(_thrust_pid.integral,
                                           -_thrust_pid.max_integral,
                                           _thrust_pid.max_integral);
    
    float output = _thrust_pid.kp * error +
                   _thrust_pid.ki * _thrust_pid.integral;
    
    return Utils::constrain(output + current_thrust, _thrust_pid.min_output, _thrust_pid.max_output);
}

// ============================================
// GoAroundController Implementation
// ============================================
GoAroundController::GoAroundController()
    : _is_active(false), _time_since_activation(0.0f),
      _phase(GoAroundPhase::INITIAL_ROTATION),
      _initial_pitch_rate(3.0f), _target_vs(0.0f),
      _target_speed(150.0f), _target_pitch(15.0f * DEG_TO_RAD),
      _target_heading(0.0f),
      _pitch_cmd(0.0f), _roll_cmd(0.0f),
      _allow_flap_retraction(false), _allow_gear_retraction(false)
{
    // A320 俯仰控制器 - 复飞时需要较激进但平滑的响应
    _pitch_controller = {
        1.5f,   // kp - 比例增益
        0.3f,   // ki - 积分增益
        0.2f,   // kd - 微分增益
        0.0f,   // integral
        0.0f    // last_error
    };
    
    // A320 滚转控制器 - 保持航向稳定
    _roll_controller = {
        1.0f,   // kp
        0.1f,   // ki
        0.15f,  // kd
        0.0f,   // integral
        0.0f    // last_error
    };
}

void GoAroundController::activate(const AircraftState& state) {
    _is_active = true;
    _time_since_activation = 0.0f;
    _phase = GoAroundPhase::INITIAL_ROTATION;

    // 保存当前航向 - 复飞时保持当前航向
    _target_heading = state.yaw * RAD_TO_DEG;

    // 初始化目标速度 - V2 + 10 (典型 A320 复飞速度)
    // 如果当前速度高于 V2，则保持当前速度；否则使用 V2+10
    float current_speed_knots = state.cas * MS_TO_KNOTS;
    _target_speed = std::max(current_speed_knots, 150.0f); // 至少 150 节

    // 初始目标俯仰角 - 从当前俯仰平滑过渡到 15°
    _target_pitch = state.pitch; // 先保持当前俯仰

    // 目标爬升率
    _target_vs = 2000.0f; // fpm

    // 记录初始离地高度（优先使用 hagl，如果为 0 或负值则退化为 alt_msl）
    _initial_altitude_ft = state.hagl > 0.0f ? state.hagl / 0.3048f : state.alt_msl / 0.3048f;

    // 重置积分器
    _pitch_controller.integral = 0.0f;
    _pitch_controller.last_error = 0.0f;
    _roll_controller.integral = 0.0f;
    _roll_controller.last_error = 0.0f;

    // 重置指令
    _pitch_cmd = state.pitch;
    _roll_cmd = state.roll;

    // 初始阶段不允许收襟翼和起落架
    _allow_flap_retraction = false;
    _allow_gear_retraction = false;

    // 标志位：还未达到正爬升率（用于起落架收放顺序的安全判断）
    _positive_climb_established = false;
}

void GoAroundController::reset() {
    _is_active = false;
    _time_since_activation = 0.0f;
    _phase = GoAroundPhase::INITIAL_ROTATION;
}

void GoAroundController::update(const AircraftState& state, float dt) {
    if (!_is_active) return;
    
    _time_since_activation += dt;
    
    // 1. 更新阶段状态
    updatePhase(state);
    
    // 2. 计算俯仰控制指令
    _pitch_cmd = computePitch(state, dt);
    
    // 3. 计算滚转控制指令（保持航向）
    _roll_cmd = computeRoll(state, dt);
}

const char* GoAroundController::getPhaseName() const {
    switch (_phase) {
        case GoAroundPhase::INITIAL_ROTATION: return "Initial Rotation";
        case GoAroundPhase::CLIMB_OUT_V2: return "Climb Out (V2+10)";
        case GoAroundPhase::ACCELERATION: return "Acceleration";
        case GoAroundPhase::CLIMB_OUT: return "Climb Out";
        case GoAroundPhase::TRANSITION_TO_CLIMB: return "Transition to Climb";
        default: return "Unknown";
    }
}

void GoAroundController::updatePhase(const AircraftState& state) {
    // 优先使用离地高度 (hagl)，否则退化到标准气压高度
    float altitude_ft = (state.hagl > 0.0f) ? (state.hagl / 0.3048f) : (state.alt_msl / 0.3048f);

    // --- 正爬升率检测（收轮的先决条件） ---
    // 用 pitch 和速度估算垂直速度：vz ≈ sin(pitch) * TAS
    float vz_mps = std::sin(state.pitch) * state.tas;
    float vz_fpm = vz_mps / 0.3048f * 60.0f;
    if (vz_fpm > 100.0f) {
        _positive_climb_established = true;
    }

    switch (_phase) {
        case GoAroundPhase::INITIAL_ROTATION: {
            // 0-3 秒：初始抬头，快速建立爬升姿态
            // 或者当俯仰角 > 10° 时认为初始抬头完成
            if (_time_since_activation > 3.0f || state.pitch > 10.0f * DEG_TO_RAD) {
                _phase = GoAroundPhase::CLIMB_OUT_V2;
            }
            break;
        }

        case GoAroundPhase::CLIMB_OUT_V2: {
            // 离地到 400ft：保持 V2+10 速度为主，爬升姿态稳定
            // 400ft 以下严禁收襟翼；正爬升率建立后允许收轮（由外部查询）
            if (altitude_ft > 400.0f) {
                _phase = GoAroundPhase::ACCELERATION;
                _allow_flap_retraction = true; // 允许收襟翼
            }
            break;
        }

        case GoAroundPhase::ACCELERATION: {
            // 400ft - 1500ft：加速阶段，准备收襟翼和起落架
            // 目标速度从 V2+10 平滑增加到 220kts 左右
            if (altitude_ft > 1500.0f) {
                _phase = GoAroundPhase::CLIMB_OUT;
                _allow_gear_retraction = true;  // 允许收起落架（实际上应该更早，这里保守处理）
                _target_speed = 220.0f;
            } else {
                // 在加速阶段平滑提升目标速度
                float t = (altitude_ft - 400.0f) / 1100.0f;
                t = Utils::constrain(t, 0.0f, 1.0f);
                // 初始 150kts → 220kts
                float base_speed = 150.0f;
                _target_speed = base_speed + (220.0f - base_speed) * t;
            }
            break;
        }

        case GoAroundPhase::CLIMB_OUT: {
            // 1500ft - 3000ft：正常爬升，速度进一步增加到 250 节
            if (altitude_ft > 3000.0f) {
                _phase = GoAroundPhase::TRANSITION_TO_CLIMB;
                _target_speed = 250.0f;
                _target_vs = 1500.0f; // 降低爬升率目标，过渡到正常爬升
            } else {
                float t = (altitude_ft - 1500.0f) / 1500.0f;
                t = Utils::constrain(t, 0.0f, 1.0f);
                _target_speed = 220.0f + (250.0f - 220.0f) * t;
            }
            break;
        }

        case GoAroundPhase::TRANSITION_TO_CLIMB: {
            // 3000ft 以上：过渡完成，不再改变内部状态，交由外部切换模式
            break;
        }
    }
}

float GoAroundController::computePitch(const AircraftState& state, float dt) {
    // 计算目标俯仰角
    float target_pitch = computeTargetPitch(state);

    // 计算俯仰误差
    float pitch_error = target_pitch - state.pitch;

    // 使用 PID 控制器计算俯仰指令
    float proportional = _pitch_controller.kp * pitch_error;

    _pitch_controller.integral += pitch_error * dt;
    // 积分饱和限制
    _pitch_controller.integral = Utils::constrain(
        _pitch_controller.integral, -10.0f * DEG_TO_RAD, 10.0f * DEG_TO_RAD);
    float integral = _pitch_controller.ki * _pitch_controller.integral;

    float derivative = 0.0f;
    if (dt > 0.001f) {
        derivative = _pitch_controller.kd * (pitch_error - _pitch_controller.last_error) / dt;
    }
    _pitch_controller.last_error = pitch_error;

    // 计算目标俯仰增量（PID 输出）
    float pitch_increment = proportional + integral + derivative;

    // 应用俯仰速率限制（3°/s）
    float max_pitch_rate_deg_s = 3.0f;
    pitch_increment = Utils::limitRate(0.0f, pitch_increment, max_pitch_rate_deg_s * DEG_TO_RAD, dt);

    // 计算最终俯仰指令
    float pitch_cmd = state.pitch + pitch_increment;

    // 应用复飞阶段的俯仰限制
    pitch_cmd = limitPitchForGoAround(pitch_cmd, state);

    return pitch_cmd;
}

float GoAroundController::computeRoll(const AircraftState& state, float dt) {
    // 计算目标滚转角 - 基于航向误差
    float heading_error = Utils::wrapAngle(_target_heading - state.yaw * RAD_TO_DEG);

    // 航向误差 -> 目标滚转角
    // A320 典型：每度航向误差约产生 2° 滚转角指令，最大 25°
    float target_roll = Utils::constrain(heading_error * 2.0f, -25.0f, 25.0f) * DEG_TO_RAD;

    // 计算滚转误差
    float roll_error = target_roll - state.roll;

    // PID 控制
    float proportional = _roll_controller.kp * roll_error;

    _roll_controller.integral += roll_error * dt;
    _roll_controller.integral = Utils::constrain(
        _roll_controller.integral, -10.0f * DEG_TO_RAD, 10.0f * DEG_TO_RAD);
    float integral = _roll_controller.ki * _roll_controller.integral;

    float derivative = 0.0f;
    if (dt > 0.001f) {
        derivative = _roll_controller.kd * (roll_error - _roll_controller.last_error) / dt;
    }
    _roll_controller.last_error = roll_error;

    // 计算滚转增量（PID 输出）
    float roll_increment = proportional + integral + derivative;

    // 应用滚转角速率限制（5°/s）
    float max_roll_rate_deg_s = 5.0f;
    roll_increment = Utils::limitRate(0.0f, roll_increment, max_roll_rate_deg_s * DEG_TO_RAD, dt);

    // 计算最终滚转指令
    float roll_cmd = state.roll + roll_increment;

    // 滚转角绝对值限制（25°）
    float max_roll = 25.0f * DEG_TO_RAD;
    roll_cmd = Utils::constrain(roll_cmd, -max_roll, max_roll);

    return roll_cmd;
}

float GoAroundController::computeTargetPitch(const AircraftState& state) {
    float current_speed_knots = state.cas * MS_TO_KNOTS;
    float speed_error = _target_speed - current_speed_knots;
    
    // 计算飞行路径角（FPA）- 假设目标FPA = 目标垂直速度/地速
    float groundspeed_knots = state.groundspeed * MS_TO_KNOTS;
    if (groundspeed_knots < 50.0f) groundspeed_knots = 50.0f;
    float target_fpa = std::atan2(_target_vs / 60.0f, groundspeed_knots * KNOTS_TO_MS * 3.281f);
    
    // 基础目标俯仰 = 飞行路径角 + 迎角
    float base_pitch = target_fpa + state.aoa;
    
    // 阶段特定的俯仰调整
    float target_pitch;
    
    switch (_phase) {
        case GoAroundPhase::INITIAL_ROTATION: {
            // 初始阶段：快速建立俯仰角，同时检查迎角
            float time_factor = std::min(_time_since_activation / 3.0f, 1.0f);
            target_pitch = state.pitch + time_factor * (15.0f * DEG_TO_RAD - state.pitch);
            
            // 迎角保护：如果接近 alpha_prot，减小俯仰
            if (state.aoa > state.alpha_prot * 0.9f) {
                target_pitch -= 2.0f * DEG_TO_RAD;
            }
            break;
        }
            
        case GoAroundPhase::CLIMB_OUT_V2: {
            // 低高度：速度优先，确保保持 V2+10
            // 计算当前垂直速度（用 pitch 和 TAS 估算）
            float current_vs_fpm = std::sin(state.pitch) * state.tas / 0.3048f * 60.0f;
            float vs_error = _target_vs - current_vs_fpm;
            float pitch_for_vs = computePitchForVS(vs_error);
            float pitch_for_speed = computePitchForSpeed(speed_error);

            // 速度过低时，优先保持速度（减小俯仰以加速）
            if (speed_error > 10.0f) {
                // 速度比目标低10节以上，减小俯仰
                target_pitch = blendPitchCommands(pitch_for_vs, pitch_for_speed, speed_error);
            } else {
                target_pitch = std::max(base_pitch, pitch_for_vs);
            }

            // 硬限制：最大15°（FCOM 推荐）
            target_pitch = std::min(target_pitch, 15.0f * DEG_TO_RAD);
            break;
        }

        case GoAroundPhase::ACCELERATION: {
            // 加速阶段：保持稳定爬升率，允许加速
            float current_vs_fpm = std::sin(state.pitch) * state.tas / 0.3048f * 60.0f;
            float vs_error = _target_vs - current_vs_fpm;
            float pitch_for_vs = computePitchForVS(vs_error);

            // 允许速度逐渐增加
            target_pitch = pitch_for_vs;

            // 如果速度仍低于目标，减小俯仰以加速
            if (speed_error > 5.0f) {
                target_pitch -= (speed_error - 5.0f) * 0.1f * DEG_TO_RAD;
            }

            // 硬限制：最大15°
            target_pitch = std::min(target_pitch, 15.0f * DEG_TO_RAD);
            break;
        }

        case GoAroundPhase::CLIMB_OUT:
        case GoAroundPhase::TRANSITION_TO_CLIMB: {
            // 高高度：正常爬升控制
            // 以垂直速度为主，速度为辅
            float current_vs_fpm = std::sin(state.pitch) * state.tas / 0.3048f * 60.0f;
            float vs_error = _target_vs - current_vs_fpm;
            float pitch_for_vs = computePitchForVS(vs_error);
            float pitch_for_speed = computePitchForSpeed(speed_error);

            // 混合两个需求
            target_pitch = blendPitchCommands(pitch_for_vs, pitch_for_speed, speed_error);

            // 限制俯仰范围
            target_pitch = Utils::constrain(target_pitch, -5.0f * DEG_TO_RAD, 15.0f * DEG_TO_RAD);
            break;
        }
            
        default:
            target_pitch = base_pitch;
    }
    
    return target_pitch;
}

float GoAroundController::computePitchForSpeed(float speed_error_knots) {
    // 速度误差 -> 俯仰修正
    // 正误差（速度过低）：需要减小俯仰以加速，或保持当前
    // 负误差（速度过高）：可以增加俯仰
    
    float pitch_correction;
    
    if (speed_error_knots > 0) {
        // 速度低于目标：减小俯仰（但不要俯冲）
        pitch_correction = -std::min(speed_error_knots * 0.3f, 10.0f) * DEG_TO_RAD;
    } else {
        // 速度高于目标：可以适度增加俯仰（但不要超过安全限制）
        pitch_correction = std::min(-speed_error_knots * 0.2f, 5.0f) * DEG_TO_RAD;
    }
    
    return pitch_correction;
}

float GoAroundController::computePitchForVS(float vs_error_fpm) {
    // 垂直速度误差 -> 俯仰修正
    // A320 典型：每 100 fpm 误差约对应 1° 俯仰修正
    
    float pitch_correction = (vs_error_fpm / 100.0f) * 1.0f * DEG_TO_RAD;
    
    // 限制单次修正
    pitch_correction = Utils::constrain(pitch_correction, -10.0f * DEG_TO_RAD, 10.0f * DEG_TO_RAD);
    
    return pitch_correction;
}

float GoAroundController::computePitchForAltitude(float altitude_error_ft) {
    // 高度误差 -> 俯仰修正
    // 每 100ft 高度误差约 1° 俯仰修正
    float pitch_correction = (altitude_error_ft / 100.0f) * 1.0f * DEG_TO_RAD;
    
    pitch_correction = Utils::constrain(pitch_correction, -10.0f * DEG_TO_RAD, 10.0f * DEG_TO_RAD);
    
    return pitch_correction;
}

float GoAroundController::blendPitchCommands(float pitch_for_vs, float pitch_for_speed, float speed_error) {
    // 根据速度误差大小混合俯仰指令
    float abs_error = std::abs(speed_error);
    
    // 速度误差 < 5节：90% 垂直速度控制，10% 速度控制
    // 速度误差 > 20节：20% 垂直速度控制，80% 速度控制
    float vs_weight;
    float speed_weight;
    
    if (abs_error < 5.0f) {
        vs_weight = 0.9f;
        speed_weight = 0.1f;
    } else if (abs_error > 20.0f) {
        vs_weight = 0.2f;
        speed_weight = 0.8f;
    } else {
        float t = (abs_error - 5.0f) / 15.0f;
        vs_weight = 0.9f - 0.7f * t;
        speed_weight = 0.1f + 0.7f * t;
    }
    
    return vs_weight * pitch_for_vs + speed_weight * pitch_for_speed;
}

float GoAroundController::limitPitchForGoAround(float pitch, const AircraftState& state) {
    // A320 复飞程序的俯仰限制
    
    // 1. 最大上仰角 - 一般 15°，但根据阶段可调
    float max_pitch_up = 15.0f;
    if (_phase == GoAroundPhase::INITIAL_ROTATION) {
        max_pitch_up = 18.0f; // 初始阶段允许稍大一些
    }
    
    // 2. 最大下俯角 - 复飞时一般不下俯
    float max_pitch_down = 5.0f; // 最多 5° 下俯
    
    // 3. 迎角保护 - 基于 alpha_prot
    if (state.alpha_prot > 0) {
        // 限制俯仰使迎角不超过 alpha_prot - 2° 裕度
        float aoa_limit = state.alpha_prot - 2.0f * DEG_TO_RAD;
        float max_pitch_for_aoa = state.pitch - (state.aoa - aoa_limit);
        max_pitch_up = std::min(max_pitch_up, max_pitch_for_aoa * RAD_TO_DEG);
    }
    
    // 4. 速度相关的俯仰限制
    float current_speed_knots = state.cas * MS_TO_KNOTS;
    if (current_speed_knots < 140.0f) {
        // 低速时进一步限制俯仰
        max_pitch_up = std::min(max_pitch_up, 12.0f);
    }
    
    // 应用限制（转换弧度进行比较）
    float pitch_deg = pitch * RAD_TO_DEG;
    pitch_deg = Utils::constrain(pitch_deg, -max_pitch_down, max_pitch_up);
    
    return pitch_deg * DEG_TO_RAD;
}

float GoAroundController::limitPitchRate(float pitch_rate, float max_rate_deg_s) {
    float max_rate = max_rate_deg_s * DEG_TO_RAD;
    return Utils::constrain(pitch_rate, -max_rate, max_rate);
}

// ============================================
// FlightControlComputer Implementation
// ============================================
FlightControlComputer::FlightControlComputer()
    : _current_mode(AFMode::OFF),
      _target_altitude(35000.0f),
      _target_vs(0.0f),
      _target_heading(0.0f),
      _target_speed(250.0f),
      _target_mach(0.78f),
      _altitude_hold_active(false),
      _speed_hold_active(false),
      _heading_hold_active(false),
      _altitude_integral(0.0f),
      _vs_integral(0.0f),
      _speed_integral(0.0f),
      _pitch_integral(0.0f),
      _roll_integral(0.0f)
{
    // A320典型控制增益
    _gains = {
        2.0f, 0.05f, 0.5f,   // pitch
        1.5f, 0.02f, 0.3f,   // roll
        0.5f,                 // yaw
        0.002f, 0.0001f,      // altitude
        0.5f, 0.02f,          // vertical speed
        0.1f, 0.005f          // speed
    };
}

void FlightControlComputer::setAFMode(AFMode mode) {
    _current_mode = mode;
    
    // 重置积分器
    _altitude_integral = 0.0f;
    _vs_integral = 0.0f;
    _pitch_integral = 0.0f;
    _roll_integral = 0.0f;
    
    // 更新模式标志
    _altitude_hold_active = (mode == AFMode::ALTITUDE_HOLD);
    _speed_hold_active = (mode == AFMode::SPEED_HOLD || mode == AFMode::MACH_HOLD);
    _heading_hold_active = (mode == AFMode::HEADING_HOLD);
}

void FlightControlComputer::setTargetAltitude(float alt_feet) {
    _target_altitude = alt_feet;
    _altitude_integral = 0.0f;
}

void FlightControlComputer::setTargetVerticalSpeed(float fpm) {
    _target_vs = fpm;
    _vs_integral = 0.0f;
}

void FlightControlComputer::setTargetHeading(float hdg_deg) {
    _target_heading = Utils::wrapAngle(hdg_deg);
}

void FlightControlComputer::setTargetSpeed(float knots) {
    _target_speed = knots;
    _speed_integral = 0.0f;
}

void FlightControlComputer::setTargetMach(float mach) {
    _target_mach = mach;
}

void FlightControlComputer::activateGoAround(const AircraftState& state) {
    setAFMode(AFMode::GO_AROUND);
    _target_vs = 2000.0f;  // GO AROUND 爬升率
    _target_speed = 150.0f; // GO AROUND 速度
    _go_around.activate(state); // 初始化复飞控制器
}

ControlCommand FlightControlComputer::computeControl(const AircraftState& state) {
    ControlCommand cmd = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    
    if (_current_mode == AFMode::OFF) {
        return cmd;
    }
    
    switch (_current_mode) {
        case AFMode::ATTITUDE_HOLD:
            cmd.pitch_cmd = attitudeHoldControl(state);
            cmd.roll_cmd = 0.0f;
            break;
            
        case AFMode::HEADING_HOLD:
            cmd.pitch_cmd = 0.0f;
            cmd.roll_cmd = headingHoldControl(state);
            break;
            
        case AFMode::ALTITUDE_HOLD:
            cmd.pitch_cmd = altitudeHoldControl(state);
            cmd.roll_cmd = 0.0f;
            break;
            
        case AFMode::VERTICAL_SPEED:
            cmd.pitch_cmd = verticalSpeedControl(state);
            cmd.roll_cmd = 0.0f;
            break;
            
        case AFMode::SPEED_HOLD:
            // 速度保持通过自动油门实现，俯仰保持水平
            cmd.pitch_cmd = 0.0f;
            cmd.roll_cmd = 0.0f;
            break;
            
        case AFMode::MACH_HOLD:
            cmd.pitch_cmd = 0.0f;
            cmd.roll_cmd = 0.0f;
            break;
            
        case AFMode::CLIMB:
            cmd.pitch_cmd = verticalSpeedControl(state);
            cmd.roll_cmd = 0.0f;
            break;
            
        case AFMode::DESCENT:
            cmd.pitch_cmd = verticalSpeedControl(state);
            cmd.roll_cmd = 0.0f;
            break;
            
        case AFMode::GO_AROUND:
            // 复飞模式：俯仰与滚转指令均由 GoAroundController 统一计算
            cmd.pitch_cmd = goAroundControl(state);
            cmd.roll_cmd  = _go_around.getRollCommand();
            break;
    }
    
    // 应用限制
    cmd.pitch_cmd = limitPitch(cmd.pitch_cmd);
    cmd.roll_cmd = limitRoll(cmd.roll_cmd);
    
    return cmd;
}

float FlightControlComputer::altitudeHoldControl(const AircraftState& state) {
    float current_alt_feet = metersToFeet(state.alt_std);
    float error = _target_altitude - current_alt_feet;
    
    float dt = 0.05f;
    
    _altitude_integral += error * dt;
    _altitude_integral = Utils::constrain(_altitude_integral, -10000.0f, 10000.0f);
    
    float pitch_cmd = _gains.altitude_kp * error +
                      _gains.altitude_ki * _altitude_integral;
    
    return pitch_cmd * DEG_TO_RAD;
}

float FlightControlComputer::verticalSpeedControl(const AircraftState& state) {
    // 使用姿态 + 真空速估算垂直速度（替代 state.velocity_ned）
    // vz ≈ sin(pitch) * TAS
    float vz_mps = std::sin(state.pitch) * state.tas;
    float current_vs_fpm = vz_mps / 0.3048f * 60.0f;
    float error = _target_vs - current_vs_fpm;

    float dt = 0.05f;

    _vs_integral += error * dt;
    _vs_integral = Utils::constrain(_vs_integral, -5000.0f, 5000.0f);

    float pitch_cmd = _gains.vs_kp * error +
                      _gains.vs_ki * _vs_integral;

    return limitPitch(pitch_cmd * DEG_TO_RAD);
}

float FlightControlComputer::speedHoldControl(const AircraftState& state) {
    float current_speed = state.cas * MS_TO_KNOTS;
    float error = _target_speed - current_speed;
    
    float dt = 0.05f;
    
    _speed_integral += error * dt;
    _speed_integral = Utils::constrain(_speed_integral, -500.0f, 500.0f);
    
    // 速度保持通过调整俯仰实现（间接）
    // 通常由自动油门负责速度控制
    float pitch_cmd = _gains.speed_kp * error +
                      _gains.speed_ki * _speed_integral;
    
    return limitPitch(pitch_cmd * DEG_TO_RAD);
}

float FlightControlComputer::attitudeHoldControl(const AircraftState& state) {
    // 姿态保持：保持激活时的俯仰角（存储在 _target_pitch 中）
    // 如果未设置目标俯仰，则保持当前姿态
    static float stored_pitch = 0.0f;
    static bool pitch_stored = false;

    // 首次进入姿态保持模式时，记录当前俯仰角作为目标
    if (!pitch_stored || _current_mode != AFMode::ATTITUDE_HOLD) {
        stored_pitch = state.pitch;
        pitch_stored = true;
    }

    float target_pitch = stored_pitch;
    float error = target_pitch - state.pitch;

    float dt = 0.05f;
    _pitch_integral += error * dt;
    _pitch_integral = Utils::constrain(_pitch_integral, -0.5f, 0.5f);

    float pitch_cmd = _gains.pitch_kp * error +
                      _gains.pitch_ki * _pitch_integral -
                      _gains.pitch_kd * state.pitch_rate;

    return limitPitch(pitch_cmd);
}

float FlightControlComputer::headingHoldControl(const AircraftState& state) {
    float current_hdg = state.yaw * RAD_TO_DEG;
    float error = Utils::wrapAngle(_target_heading - current_hdg);
    
    float dt = 0.05f;
    _roll_integral += error * dt;
    _roll_integral = Utils::constrain(_roll_integral, -10.0f, 10.0f);
    
    float roll_cmd = _gains.roll_kp * error +
                     _gains.roll_ki * _roll_integral -
                     _gains.roll_kd * state.roll_rate;
    
    return limitRoll(roll_cmd * DEG_TO_RAD);
}

float FlightControlComputer::goAroundControl(const AircraftState& state) {
    // 让 GoAroundController 推进状态机（阶段管理 + PID 计算）
    // 注意：此处假设 50Hz 调用，如果主循环频率不同，应将 dt 作为参数传入
    static constexpr float dt = 1.0f / 50.0f;
    _go_around.update(state, dt);
    return _go_around.getPitchCommand();
}

float FlightControlComputer::limitPitch(float pitch) const {
    float max_pitch_up = 25.0f * DEG_TO_RAD;
    float max_pitch_down = 15.0f * DEG_TO_RAD;
    return Utils::constrain(pitch, -max_pitch_down, max_pitch_up);
}

float FlightControlComputer::limitRoll(float roll) const {
    float max_roll = 33.0f * DEG_TO_RAD;
    return Utils::constrain(roll, -max_roll, max_roll);
}

float FlightControlComputer::limitRate(float rate, float max_rate) const {
    return Utils::constrain(rate, -max_rate, max_rate);
}

// ============================================
// FlightManagementSystem Implementation
// ============================================
FlightManagementSystem::FlightManagementSystem()
    : _flight_phase(FlightPhase::GROUND),
      _current_waypoint(0),
      _target_altitude(0.0f),
      _target_speed(0.0f),
      _target_heading(0.0f)
{
}

void FlightManagementSystem::setFlightPhase(FlightPhase phase) {
    _flight_phase = phase;
    updateClimbProfile();
}

void FlightManagementSystem::setDepartureAirport(const std::string& icao) {
    _departure_airport = icao;
}

void FlightManagementSystem::setDestinationAirport(const std::string& icao) {
    _destination_airport = icao;
}

void FlightManagementSystem::loadRoute(const std::vector<std::string>& waypoints) {
    _route_waypoints = waypoints;
    _current_waypoint = 0;
}

void FlightManagementSystem::update(const AircraftState& state) {
    switch (_flight_phase) {
        case FlightPhase::CLIMB:
            updateClimbProfile();
            break;
        case FlightPhase::CRUISE:
            updateCruiseProfile();
            break;
        case FlightPhase::DESCENT:
            updateDescentProfile();
            break;
        default:
            break;
    }
}

void FlightManagementSystem::updateClimbProfile() {
    _target_altitude = 35000.0f;
    _target_speed = 280.0f;
    _target_heading = 0.0f;
}

void FlightManagementSystem::updateCruiseProfile() {
    _target_altitude = 38000.0f;
    _target_speed = 0.0f; // 使用马赫数
    _target_heading = 0.0f;
}

void FlightManagementSystem::updateDescentProfile() {
    _target_altitude = 10000.0f;
    _target_speed = 250.0f;
    _target_heading = 0.0f;
}

// ============================================
// AutoFlightSystem Implementation
// ============================================
AutoFlightSystem::AutoFlightSystem()
    : _autopilot_enabled(false)
{
}

void AutoFlightSystem::initialize() {
    _control_command = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
}

void AutoFlightSystem::update(const AircraftState& state) {
    if (!_autopilot_enabled) {
        return;
    }
    
    handleModeTransitions(state);
    enforceFlightEnvelope(state);
    
    // 计算飞行控制
    _control_command = _fcc.computeControl(state);
    
    // 计算自动油门
    if (_autothrottle.isEnabled()) {
        _control_command.throttle_cmd = _autothrottle.computeThrottle(state);
    }
}

void AutoFlightSystem::enableAutopilot() {
    _autopilot_enabled = true;
}

void AutoFlightSystem::disableAutopilot() {
    _autopilot_enabled = false;
}

void AutoFlightSystem::enableAutothrottle() {
    _autothrottle.setMode(true);
}

void AutoFlightSystem::disableAutothrottle() {
    _autothrottle.setMode(false);
}

void AutoFlightSystem::setAFMode(AFMode mode) {
    _fcc.setAFMode(mode);
}

void AutoFlightSystem::setTargetAltitude(float feet) {
    _fcc.setTargetAltitude(feet);
}

void AutoFlightSystem::setTargetVerticalSpeed(float fpm) {
    _fcc.setTargetVerticalSpeed(fpm);
}

void AutoFlightSystem::setTargetHeading(float deg) {
    _fcc.setTargetHeading(deg);
}

void AutoFlightSystem::setTargetSpeed(float knots) {
    _fcc.setTargetSpeed(knots);
    _autothrottle.setTargetSpeed(knots);
}

void AutoFlightSystem::setTargetMach(float mach) {
    _fcc.setTargetMach(mach);
    _autothrottle.setTargetMach(mach);
}

void AutoFlightSystem::activateGoAround(const AircraftState& state) {
    _fcc.activateGoAround(state);            // 初始化复飞状态机（需要 state）
    _autothrottle.setFlightPhase(FlightPhase::GO_AROUND);
    // 同步自动油门的目标速度（复飞阶段虽用固定推力，但过渡后需要）
    _autothrottle.setTargetSpeed(_fcc.getGoAroundController().getTargetSpeed());
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

void AutoFlightSystem::handleModeTransitions(const AircraftState& state) {
    // 自动模式切换逻辑
    AFMode current_mode = _fcc.getAFMode();

    if (current_mode == AFMode::GO_AROUND) {
        // 复飞状态机完成后切换到 V/S 爬升模式
        if (_fcc.getGoAroundController().isCompleted()) {
            setAFMode(AFMode::VERTICAL_SPEED);
            // 使用复飞末期的爬升率（1500 fpm）作为过渡目标
            setTargetVerticalSpeed(_fcc.getGoAroundController().getTargetVerticalSpeed());
            setTargetSpeed(_fcc.getGoAroundController().getTargetSpeed());
        }
    }
}

void AutoFlightSystem::enforceFlightEnvelope(const AircraftState& state) {
    // 迎角保护：接近 alpha_prot 时减小俯仰角以降低迎角
    if (state.aoa > state.alpha_prot * 0.85f) {
        // 迎角过大：减小俯仰角指令（推杆），防止失速
        float aoa_margin = state.alpha_prot - state.aoa;
        float pitch_reduction = std::max(0.0f, -aoa_margin * RAD_TO_DEG * 2.0f) * DEG_TO_RAD;
        _control_command.pitch_cmd = Utils::constrain(
            _control_command.pitch_cmd - pitch_reduction,
            -15.0f * DEG_TO_RAD, 10.0f * DEG_TO_RAD);

        // 同时减小油门（避免加速导致迎角进一步增大）
        _control_command.throttle_cmd = Utils::constrain(
            _control_command.throttle_cmd - 3.0f, 0.0f, 100.0f);
    }

    // 超速保护：马赫数超过 0.86 时减小油门
    if (state.mach > 0.86f) {
        _control_command.throttle_cmd = Utils::constrain(
            _control_command.throttle_cmd - 10.0f, 0.0f, 100.0f);
    }

    // 低速保护：速度过低时增加油门
    float speed_knots = state.cas * MS_TO_KNOTS;
    if (speed_knots < 140.0f && _fcc.getAFMode() != AFMode::GO_AROUND) {
        _control_command.throttle_cmd = Utils::constrain(
            _control_command.throttle_cmd + 5.0f, 0.0f, 100.0f);
    }
}

// ============================================
// Utils Implementation
// ============================================
namespace Utils {

float wrapAngle(float angle_deg) {
    while (angle_deg >= 360.0f) angle_deg -= 360.0f;
    while (angle_deg < 0.0f) angle_deg += 360.0f;
    return angle_deg;
}

float constrain(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

float interpolate(float x, float x0, float x1, float y0, float y1) {
    if (x1 == x0) return y0;
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}

float limitRate(float current, float target, float max_rate, float dt) {
    float delta = target - current;
    float max_delta = max_rate * dt;
    delta = constrain(delta, -max_delta, max_delta);
    return current + delta;
}

} // namespace Utils

} // namespace A320