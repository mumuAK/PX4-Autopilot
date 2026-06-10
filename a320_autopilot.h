#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <string>
#include <array>

namespace A320 {

using namespace Eigen;

// 常用常量
constexpr float RAD_TO_DEG = 57.2958f;
constexpr float DEG_TO_RAD = 0.0174533f;
constexpr float GRAVITY = 9.81f;
constexpr float KNOTS_TO_MS = 0.514444f;
constexpr float MS_TO_KNOTS = 1.94384f;

// 飞行器状态
struct AircraftState {
    // 位置
    double lat;           // 纬度 (rad)
    double lon;           // 经度 (rad)
    float alt_msl;        // 海拔高度 (m)
    float alt_std;        // 标准气压高度 (m)
    float hagl;           // 离地高度 (m)
    
    // 速度
    float tas;            // 真空速 (m/s)
    float cas;            // 校准空速 (m/s)
    float mach;           // 马赫数
    float groundspeed;    // 地速 (m/s)
    
    // 姿态
    float roll;           // 滚转角 (rad)
    float pitch;          // 俯仰角 (rad)
    float yaw;            // 航向角 (rad)
    
    // 角速度
    float roll_rate;      // 滚转角速度 (rad/s)
    float pitch_rate;     // 俯仰角速度 (rad/s)
    float yaw_rate;       // 航向角速度 (rad/s)
    
    // 加速度
    float normal_accel;    // 法向加速度 (g)
    float forward_accel;   // 纵向加速度 (g)
    
    // 发动机状态
    float n1_left;        // 左发N1转速 (%)
    float n1_right;       // 右发N1转速 (%)
    float thrust_left;    // 左发推力 (N)
    float thrust_right;   // 右发推力 (N)
    float epr_left;       // 左发EPR
    float epr_right;      // 右发EPR
    
    // 飞行状态
    bool is_armed;        // 发动机启动
    bool is_in_air;       // 是否在空中
    bool is_gear_down;    // 起落架放下
    bool is_flaps_extended; // 襟翼伸出
    
    // 大气数据
    float aoa;            // 迎角 (rad)
    float alpha_prot;     // 迎角保护值 (rad)
    float beta;           // 侧滑角 (rad)
    float qnh;            // 修正海压 (hPa)
    float temperature;    // 外界温度 (°C)
    float pressure_alt;   // 气压高度 (m)
};

// 控制指令
struct ControlCommand {
    float roll_cmd;       // 滚转角指令 (rad)
    float pitch_cmd;      // 俯仰角指令 (rad)
    float yaw_cmd;        // 航向角指令 (rad)
    float throttle_cmd;   // 油门指令 (0-100%)
    float flap_cmd;       // 襟翼指令
    float gear_cmd;       // 起落架指令
};

// 自动飞行模式
enum class AFMode {
    OFF,
    ATTITUDE_HOLD,
    HEADING_HOLD,
    ALTITUDE_HOLD,
    VERTICAL_SPEED,
    SPEED_HOLD,
    MACH_HOLD,
    CLIMB,
    DESCENT,
    GO_AROUND
};

// 飞行阶段
enum class FlightPhase {
    GROUND,
    TAKEOFF,
    CLIMB,
    CRUISE,
    DESCENT,
    APPROACH,
    LANDING,
    GO_AROUND
};

// ============================================
// 自动油门系统 (Autothrottle)
// ============================================
class Autothrottle {
public:
    Autothrottle();
    
    void setMode(bool enabled);
    bool isEnabled() const { return _enabled; }
    
    void setTargetSpeed(float speed_knots);
    void setTargetMach(float mach);
    void setTargetThrust(float n1_percent);
    void setTargetEPR(float epr);
    
    void setFlightPhase(FlightPhase phase);
    
    float computeThrottle(const AircraftState& state);
    
private:
    bool _enabled;
    float _target_speed;    // 目标空速 (knots)
    float _target_mach;     // 目标马赫数
    float _target_thrust;   // 目标推力 (%)
    float _target_epr;      // 目标EPR
    
    FlightPhase _flight_phase;
    
    // PID控制器参数
    struct PID {
        float kp;
        float ki;
        float kd;
        float integral;
        float max_integral;
        float min_output;
        float max_output;
    };
    
    PID _speed_pid;
    PID _mach_pid;
    PID _thrust_pid;
    
    float _last_speed_error;
    float _last_time;
    
    float computeSpeedControl(const AircraftState& state);
    float computeMachControl(const AircraftState& state);
    float computeThrustControl(const AircraftState& state);
};

// ============================================
// 飞行控制计算机 (FlightControlComputer)
// ============================================
class FlightControlComputer {
public:
    FlightControlComputer();
    
    void setAFMode(AFMode mode);
    AFMode getAFMode() const { return _current_mode; }
    
    void setTargetAltitude(float alt_feet);
    void setTargetVerticalSpeed(float fpm);
    void setTargetHeading(float hdg_deg);
    void setTargetSpeed(float knots);
    void setTargetMach(float mach);
    
    void activateGoAround();
    bool isGoAroundActive() const { return _current_mode == AFMode::GO_AROUND; }
    
    ControlCommand computeControl(const AircraftState& state);
    
private:
    AFMode _current_mode;
    
    // 目标值
    float _target_altitude;   // 目标高度 (ft)
    float _target_vs;         // 目标垂直速度 (fpm)
    float _target_heading;    // 目标航向 (deg)
    float _target_speed;      // 目标空速 (knots)
    float _target_mach;       // 目标马赫数
    
    // 控制模式管理器
    bool _altitude_hold_active;
    bool _speed_hold_active;
    bool _heading_hold_active;
    
    // 控制参数
    struct ControlGains {
        float pitch_kp;
        float pitch_ki;
        float pitch_kd;
        float roll_kp;
        float roll_ki;
        float roll_kd;
        float yaw_kp;
        float altitude_kp;
        float altitude_ki;
        float vs_kp;
        float vs_ki;
        float speed_kp;
        float speed_ki;
    };
    
    ControlGains _gains;
    
    // 积分器状态
    float _altitude_integral;
    float _vs_integral;
    float _speed_integral;
    float _pitch_integral;
    float _roll_integral;
    
    // 计算各轴控制
    float computePitchCommand(const AircraftState& state);
    float computeRollCommand(const AircraftState& state);
    float computeYawCommand(const AircraftState& state);
    
    // 模式控制
    float altitudeHoldControl(const AircraftState& state);
    float verticalSpeedControl(const AircraftState& state);
    float speedHoldControl(const AircraftState& state);
    float attitudeHoldControl(const AircraftState& state);
    float headingHoldControl(const AircraftState& state);
    float goAroundControl(const AircraftState& state);
    
    // 限制器
    float limitPitch(float pitch) const;
    float limitRoll(float roll) const;
    float limitRate(float rate, float max_rate) const;
    
    // 辅助函数
    float feetToMeters(float feet) const { return feet * 0.3048f; }
    float metersToFeet(float meters) const { return meters / 0.3048f; }
    float fpmToMs(float fpm) const { return fpm * 0.00508f; }
    float msToFpm(float ms) const { return ms / 0.00508f; }
};

// ============================================
// 飞行管理系统 (FlightManagementSystem)
// ============================================
class FlightManagementSystem {
public:
    FlightManagementSystem();
    
    void setFlightPhase(FlightPhase phase);
    FlightPhase getFlightPhase() const { return _flight_phase; }
    
    void setDepartureAirport(const std::string& icao);
    void setDestinationAirport(const std::string& icao);
    
    void loadRoute(const std::vector<std::string>& waypoints);
    
    void update(const AircraftState& state);
    
    float getTargetAltitude() const { return _target_altitude; }
    float getTargetSpeed() const { return _target_speed; }
    float getTargetHeading() const { return _target_heading; }
    
private:
    FlightPhase _flight_phase;
    
    std::string _departure_airport;
    std::string _destination_airport;
    std::vector<std::string> _route_waypoints;
    size_t _current_waypoint;
    
    float _target_altitude;
    float _target_speed;
    float _target_heading;
    
    void updateClimbProfile();
    void updateCruiseProfile();
    void updateDescentProfile();
};

// ============================================
// 自动飞行系统 (AutoFlightSystem) - 整合所有子系统
// ============================================
class AutoFlightSystem {
public:
    AutoFlightSystem();
    
    void initialize();
    void update(const AircraftState& state);
    
    // 模式控制
    void enableAutopilot();
    void disableAutopilot();
    bool isAutopilotEnabled() const { return _autopilot_enabled; }
    
    void enableAutothrottle();
    void disableAutothrottle();
    bool isAutothrottleEnabled() const { return _autothrottle.isEnabled(); }
    
    void setAFMode(AFMode mode);
    AFMode getAFMode() const { return _fcc.getAFMode(); }
    
    // 目标值设置
    void setTargetAltitude(float feet);
    void setTargetVerticalSpeed(float fpm);
    void setTargetHeading(float deg);
    void setTargetSpeed(float knots);
    void setTargetMach(float mach);
    
    // 复飞
    void activateGoAround();
    bool isGoAroundActive() const { return _fcc.isGoAroundActive(); }
    
    // 获取输出
    ControlCommand getControlCommand() const { return _control_command; }
    
private:
    bool _autopilot_enabled;
    
    Autothrottle _autothrottle;
    FlightControlComputer _fcc;
    FlightManagementSystem _fms;
    
    ControlCommand _control_command;
    
    void handleModeTransitions(const AircraftState& state);
    void enforceFlightEnvelope(const AircraftState& state);
};

// ============================================
// 辅助函数
// ============================================
namespace Utils {
    float wrapAngle(float angle_deg);
    float constrain(float value, float min, float max);
    float interpolate(float x, float x0, float x1, float y0, float y1);
    float limitRate(float current, float target, float max_rate, float dt);
}

} // namespace A320