/**
 * @file a320_ground_control.h
 * @brief A320 地面控制器
 * 
 * 实现地面滑跑时的航向控制，包括：
 * - 前轮转弯（NWS）控制（低速）
 * - 方向舵控制（高速）
 * - 差动刹车控制（极低速）
 * 
 * A320 地面控制策略：
 * - 速度 < 30 kts: 主要使用差动刹车
 * - 30-80 kts: 前轮转弯 + 方向舵混合
 * - 80-130 kts: 主要使用方向舵
 * - > 130 kts (离地后): 仅方向舵 + 副翼
 * 
 * @author AutoGen
 * @date 2026
 */

#pragma once

#include "a320_autopilot_common.h"

namespace A320 {

/**
 * @class GroundController
 * @brief 地面控制器
 * 
 * 负责地面滑跑时的航向控制，根据速度自动选择控制策略。
 */
class GroundController {
public:
    /**
     * @brief 构造函数
     */
    GroundController();

    /**
     * @brief 初始化控制器
     */
    void initialize();

    /**
     * @brief 设置目标航向
     * @param heading_deg 目标航向（度）
     */
    void setTargetHeading(float heading_deg);

    /**
     * @brief 设置跑道航向
     * @param runway_heading_deg 跑道航向（度）
     */
    void setRunwayHeading(float runway_heading_deg);

    /**
     * @brief 计算地面控制指令
     * 
     * 根据当前速度和航向误差，计算前轮转弯、方向舵和差动刹车指令。
     * 
     * @param state 当前飞机状态
     * @param dt 时间步长（秒）
     */
    void update(const AircraftState& state, float dt);

    /**
     * @brief 获取前轮转弯指令
     * @return 前轮转角（rad），正值为右转
     */
    float getNWSCommand() const { return _nws_cmd; }

    /**
     * @brief 获取方向舵指令
     * @return 方向舵偏角（rad），正值为右舵
     */
    float getRudderCommand() const { return _rudder_cmd; }

    /**
     * @brief 获取左刹车差动指令
     * @return 左刹车百分比（0-100%）
     */
    float getLeftBrakeCommand() const { return _left_brake_cmd; }

    /**
     * @brief 获取右刹车差动指令
     * @return 右刹车百分比（0-100%）
     */
    float getRightBrakeCommand() const { return _right_brake_cmd; }

    /**
     * @brief 获取当前地面控制模式
     * @return GroundControlMode 枚举值
     */
    GroundControlMode getControlMode() const { return _control_mode; }

    /**
     * @brief 获取控制模式名称
     * @return 模式名称字符串
     */
    const char* getControlModeName() const;

private:
    float _target_heading;          ///< 目标航向（度）
    float _runway_heading;          ///< 跑道航向（度）

    float _nws_cmd;                 ///< 前轮转弯指令（rad）
    float _rudder_cmd;              ///< 方向舵指令（rad）
    float _left_brake_cmd;          ///< 左刹车指令（%）
    float _right_brake_cmd;         ///< 右刹车指令（%）

    GroundControlMode _control_mode; ///< 当前控制模式

    float _heading_integral;        ///< 航向积分项
    float _yaw_rate_integral;       ///< 偏航速率积分项

    /**
     * @struct Gains
     * @brief PID 控制器增益
     */
    struct Gains {
        // 前轮转弯增益
        float nws_kp;
        float nws_ki;
        float nws_kd;

        // 方向舵增益
        float rudder_kp;
        float rudder_ki;
        float rudder_kd;

        // 差动刹车增益
        float brake_kp;
        float brake_ki;

        // 速度阈值
        float speed_nws_only;       ///< 仅前轮转弯的速度上限 (knots)
        float speed_rudder_only;    ///< 仅方向舵的速度下限 (knots)
        float speed_brake_only;     ///< 仅差动刹车的速度上限 (knots)
    } _gains;

    float _last_heading_error;      ///< 上一次航向误差

    /**
     * @brief 确定控制模式
     * 
     * 根据当前速度选择合适的控制策略。
     */
    void determineControlMode(const AircraftState& state);

    /**
     * @brief 计算航向误差
     * @return 航向误差（度）
     */
    float computeHeadingError(const AircraftState& state);

    /**
     * @brief 计算前轮转弯指令
     */
    float computeNWS(const AircraftState& state, float heading_error, float dt);

    /**
     * @brief 计算方向舵指令
     */
    float computeRudder(const AircraftState& state, float heading_error, float dt);

    /**
     * @brief 计算差动刹车指令
     */
    void computeDifferentialBrake(float heading_error, float dt);

    /**
     * @brief 限制前轮转角范围
     * 
     * A320 前轮转弯限制：低速时 ±75°，高速时 ±6°
     */
    float limitNWS(float nws, float speed_knots);

    /**
     * @brief 限制方向舵偏角
     * 
     * A320 方向舵限制：±25°
     */
    float limitRudder(float rudder);

    /**
     * @brief 混合控制指令
     * 
     * 在过渡速度区间平滑混合不同控制器的输出。
     */
    void blendControls(float speed_knots);
};

} // namespace A320