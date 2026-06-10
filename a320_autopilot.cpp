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

void FlightControlComputer::activateGoAround() {
    setAFMode(AFMode::GO_AROUND);
    _target_vs = 2000.0f;  // GO AROUND爬升率
    _target_speed = 150.0f; // GO AROUND速度
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
            cmd.pitch_cmd = goAroundControl(state);
            cmd.roll_cmd = 0.0f;
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
    float current_vs_fpm = msToFpm(state.velocity_ned(2));
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
    // 姿态保持：保持当前俯仰角
    float target_pitch = state.pitch;
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
    float pitch_cmd = 15.0f * DEG_TO_RAD; // GO AROUND俯仰角
    
    // 如果高度过低，增加俯仰
    if (state.alt_msl < 1000.0f) {
        pitch_cmd = 20.0f * DEG_TO_RAD;
    }
    
    return limitPitch(pitch_cmd);
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

void AutoFlightSystem::activateGoAround() {
    _fcc.activateGoAround();
    _autothrottle.setFlightPhase(FlightPhase::GO_AROUND);
}

void AutoFlightSystem::handleModeTransitions(const AircraftState& state) {
    // 自动模式切换逻辑
    AFMode current_mode = _fcc.getAFMode();
    
    if (current_mode == AFMode::GO_AROUND) {
        // GO AROUND完成后切换到爬升模式
        if (state.alt_msl > 1500.0f) {
            setAFMode(AFMode::VERTICAL_SPEED);
            setTargetVerticalSpeed(1500.0f);
        }
    }
}

void AutoFlightSystem::enforceFlightEnvelope(const AircraftState& state) {
    // 迎角保护
    if (state.aoa > state.alpha_prot * 0.9f) {
        // 减小油门，增加俯仰
        _control_command.throttle_cmd = Utils::constrain(
            _control_command.throttle_cmd - 5.0f, 0.0f, 100.0f);
    }
    
    // 超速保护
    if (state.mach > 0.86f) {
        _control_command.throttle_cmd = Utils::constrain(
            _control_command.throttle_cmd - 10.0f, 0.0f, 100.0f);
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