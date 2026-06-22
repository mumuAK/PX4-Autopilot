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

// 飞行器状态
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
    float yaw_rate;       // 航向角速度 (rad/s)

    float normal_accel;   // 法向加速度 (g)
    float forward_accel;  // 纵向加速度 (g)

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

// 复飞阶段
enum class GoAroundPhase {
    INITIAL_ROTATION,     // 初始抬头阶段 (0-3s)
    CLIMB_OUT_V2,         // 初始爬升，保持 V2+10 (离地 - 400ft)
    ACCELERATION,         // 加速阶段，准备收襟翼 (400ft - 1500ft)
    CLIMB_OUT,            // 正常爬升 (1500ft - 3000ft)
    TRANSITION_TO_CLIMB   // 过渡到正常爬升模式 (> 3000ft)
};

// 辅助函数
namespace Utils {
    inline float wrapAngle(float angle_deg) {
        while (angle_deg >= 360.0f) angle_deg -= 360.0f;
        while (angle_deg < 0.0f) angle_deg += 360.0f;
        return angle_deg;
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
}

} // namespace A320