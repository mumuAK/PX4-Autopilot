/**
 * @file a320_autopilot_common.h
 * @brief A320 自动驾驶系统公共类型定义
 * 
 * 包含飞机状态、控制指令、飞行模式等核心数据结构。
 * 
 * @author AutoGen
 * @date 2026
 */

#pragma once

#include <cmath>
#include <string>

namespace A320 {

// 常用常量
constexpr float RAD_TO_DEG = 57.2958f;
constexpr float DEG_TO_RAD = 0.0174533f;
constexpr float GRAVITY = 9.81f;
constexpr float KNOTS_TO_MS = 0.514444f;
constexpr float MS_TO_KNOTS = 1.94384f;

/**
 * @struct AircraftState
 * @brief 飞机状态结构体
 * 
 * 包含飞机的位置、速度、姿态、发动机状态等信息。
 */
struct AircraftState {
    double lat;           // 纬度 (rad)
    double lon;           // 经度 (rad)
    float alt_msl;        // 海拔高度 (m)
    float alt_std;        // 标准气压高度 (m)
    float hagl;           // 离地高度 (m)

    float tas;            // 真空速 (m/s)
    float cas;            // 校准空速 (m/s)
    float mach;           // 马赫数
    float groundspeed;    // 地速 (m/s)

    float roll;           // 滚转角 (rad)
    float pitch;          // 俯仰角 (rad)
    float yaw;            // 航向角 (rad)

    float roll_rate;      // 滚转角速度 (rad/s)
    float pitch_rate;     // 俯仰角速度 (rad/s)
    float yaw_rate;       // 航向角速度 (rad/s) - 地面转弯速率

    float normal_accel;   // 法向加速度 (g)
    float forward_accel;  // 纵向加速度 (g)
    float lateral_accel;  // 横向加速度 (g) - 用于侧滑检测

    float n1_left;        // 左发N1转速 (%)
    float n1_right;       // 右发N1转速 (%)
    float thrust_left;    // 左发推力 (N)
    float thrust_right;   // 右发推力 (N)
    float epr_left;       // 左发EPR
    float epr_right;      // 右发EPR

    bool is_armed;        // 发动机启动
    bool is_in_air;       // 是否在空中
    bool is_gear_down;    // 起落架放下
    bool is_flaps_extended; // 襟翼伸出

    float aoa;            // 迎角 (rad)
    float alpha_prot;     // 迎角保护值 (rad)
    float beta;           // 侧滑角 (rad)
    float qnh;            // 修正海压 (hPa)
    float temperature;    // 外界温度 (°C)
    float pressure_alt;   // 气压高度 (m)

    // --- 新增：发动机状态 ---
    bool engine_left_operating;   // 左发正常工作
    bool engine_right_operating;  // 右发正常工作
    bool engine_left_failed;      // 左发失效
    bool engine_right_failed;     // 右发失效
    float engine_failure_time;    // 发动机失效后经过的时间 (s)

    // --- 新增：地面状态 ---
    float ground_track;           // 地面轨迹角 (rad) - 实际地面航向
    float cross_track_error;      // 偏航距离 (m) - 相对于跑道中心线
    float runway_heading;         // 跑道航向 (rad)
    float wheel_speed;            // 机轮速度 (m/s) - 用于判断是否可以使用前轮转弯
    float nose_wheel_angle;       // 前轮当前转角 (rad)
    bool nose_wheel_engaged;      // 前轮转弯是否接通
};

/**
 * @struct ControlCommand
 * @brief 控制指令结构体
 * 
 * 包含所有控制面的指令，包括空中和地面控制。
 */
struct ControlCommand {
    // --- 空中控制 ---
    float roll_cmd;       // 滚转角指令 (rad) - 副翼
    float pitch_cmd;      // 俯仰角指令 (rad) - 升降舵
    float yaw_cmd;        // 航向角指令 (rad) - 方向舵（空中）

    // --- 地面控制 ---
    float rudder_cmd;     // 脚蹬指令 (rad) - 方向舵偏角，正值为右舵
    float nws_cmd;        // 前轮转弯指令 (rad) - 前轮转角，正值为右转
    float differential_brake_left;  // 左刹车差动 (%)
    float differential_brake_right; // 右刹车差动 (%)

    // --- 推力控制 ---
    float throttle_cmd;   // 油门指令 (0-100%)
    float throttle_left;  // 左发油门 (单独控制)
    float throttle_right; // 右发油门 (单独控制)

    // --- 构型控制 ---
    float flap_cmd;       // 襟翼指令
    float gear_cmd;       // 起落架指令

    // --- 单发补偿 ---
    float rudder_trim;    // 方向舵配平 (rad) - 用于单发补偿
    float aileron_trim;   // 副翼配平 (rad) - 用于单发补偿
};

/**
 * @enum AFMode
 * @brief 自动飞行模式
 */
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
    GO_AROUND,
    GROUND_TRACK,       // 地面轨迹跟踪（新增）
    ENGINE_OUT          // 单发失效模式（新增）
};

/**
 * @enum FlightPhase
 * @brief 飞行阶段
 */
enum class FlightPhase {
    GROUND,
    TAKEOFF_ROLL,       // 起飞滑跑（新增）
    TAKEOFF,
    CLIMB,
    CRUISE,
    DESCENT,
    APPROACH,
    LANDING_ROLL,       // 着陆滑跑（新增）
    LANDING,
    GO_AROUND
};

/**
 * @enum GoAroundPhase
 * @brief 复飞阶段
 */
enum class GoAroundPhase {
    INITIAL_ROTATION,
    CLIMB_OUT_V2,
    ACCELERATION,
    CLIMB_OUT,
    TRANSITION_TO_CLIMB
};

/**
 * @enum EngineFailurePhase
 * @brief 单发失效阶段
 */
enum class EngineFailurePhase {
    NONE,               // 无失效
    IMMEDIATE,          // 立即响应阶段（0-5s）
    STABILIZATION,      // 稳定阶段（5-30s）
    CLIMB_OUT,          // 爬升阶段
    CRUISE              // 巡航阶段
};

/**
 * @enum GroundControlMode
 * @brief 地面控制模式
 */
enum class GroundControlMode {
    NWS_ONLY,           // 仅前轮转弯（低速）
    NWS_RUDDER,         // 前轮+方向舵（中速）
    RUDDER_ONLY,        // 仅方向舵（高速/空中）
    DIFFERENTIAL_BRAKE  // 差动刹车（极低速）
};

// 辅助函数
namespace Utils {
    inline float wrapAngle(float angle_deg) {
        while (angle_deg >= 360.0f) angle_deg -= 360.0f;
        while (angle_deg < 0.0f) angle_deg += 360.0f;
        return angle_deg;
    }

    inline float wrapAngleRad(float angle_rad) {
        while (angle_rad >= 2.0f * M_PI) angle_rad -= 2.0f * M_PI;
        while (angle_rad < 0.0f) angle_rad += 2.0f * M_PI;
        return angle_rad;
    }

    inline float constrain(float value, float min, float max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    inline float interpolate(float x, float x0, float x1, float y0, float y1) {
        if (x1 == x0) return y0;
        return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
    }

    inline float limitRate(float current, float target, float max_rate, float dt) {
        float delta = target - current;
        float max_delta = max_rate * dt;
        delta = constrain(delta, -max_delta, max_delta);
        return current + delta;
    }

    /**
     * @brief 计算推力不对称矩
     * @param thrust_left 左发推力 (N)
     * @param thrust_right 右发推力 (N)
     * @param engine_y 发动机横向距离 (m)，A320 约 5.5m
     * @return 不对称力矩 (N·m)
     */
    inline float computeAsymmetricMoment(float thrust_left, float thrust_right, float engine_y = 5.5f) {
        return (thrust_right - thrust_left) * engine_y;
    }
}

} // namespace A320