/**
 * @file a320_engine_out.h
 * @brief A320 单发失效补偿控制器
 * 
 * 实现单发失效时的航向和姿态补偿控制，包括：
 * - 推力不对称补偿（方向舵配平）
 * - 滚转补偿（副翼配平）
 * - 速度/推力管理（单发爬升性能）
 * 
 * A320 单发失效控制策略：
 * 1. 立即响应阶段（0-5s）：快速踩舵抵消偏航
 * 2. 稳定阶段（5-30s）：建立配平，稳定姿态
 * 3. 爬升阶段：调整目标速度和推力
 * 
 * @author AutoGen
 * @date 2026
 */

#pragma once

#include "a320_autopilot_common.h"

namespace A320 {

/**
 * @class EngineOutController
 * @brief 单发失效补偿控制器
 * 
 * 负责单发失效时的航向和姿态补偿，确保飞机能够安全继续飞行。
 */
class EngineOutController {
public:
    /**
     * @brief 构造函数
     */
    EngineOutController();

    /**
     * @brief 初始化控制器
     */
    void initialize();

    /**
     * @brief 激活单发失效模式
     * 
     * 当检测到发动机失效时激活此模式。
     * 
     * @param state 当前飞机状态
     * @param failed_side 失效发动机侧（"LEFT" 或 "RIGHT"）
     */
    void activate(const AircraftState& state, const std::string& failed_side);

    /**
     * @brief 重置控制器
     */
    void reset();

    /**
     * @brief 更新控制器
     * 
     * 计算单发补偿指令。
     * 
     * @param state 当前飞机状态
     * @param dt 时间步长（秒）
     */
    void update(const AircraftState& state, float dt);

    /**
     * @brief 获取方向舵配平指令
     * @return 方向舵配平（rad），正值为右舵
     */
    float getRudderTrim() const { return _rudder_trim; }

    /**
     * @brief 获取副翼配平指令
     * @return 副翼配平（rad），正值为右滚转
     */
    float getAileronTrim() const { return _aileron_trim; }

    /**
     * @brief 获取方向舵指令（立即响应）
     * @return 方向舵指令（rad）
     */
    float getRudderCommand() const { return _rudder_cmd; }

    /**
     * @brief 获取单发目标速度
     * @return 目标速度（knots）
     */
    float getTargetSpeed() const { return _target_speed; }

    /**
     * @brief 获取单发目标推力
     * @return 目标推力（N1 %）
     */
    float getTargetThrust() const { return _target_thrust; }

    /**
     * @brief 获取当前单发失效阶段
     * @return EngineFailurePhase 枚举值
     */
    EngineFailurePhase getPhase() const { return _phase; }

    /**
     * @brief 获取失效阶段名称
     * @return 阶段名称字符串
     */
    const char* getPhaseName() const;

    /**
     * @brief 检查是否激活
     * @return true 表示单发补偿正在执行
     */
    bool isActive() const { return _is_active; }

    /**
     * @brief 检查是否稳定
     * @return true 表示飞机已稳定
     */
    bool isStabilized() const { return _phase == EngineFailurePhase::CRUISE ||
                                        _phase == EngineFailurePhase::CLIMB_OUT; }

    /**
     * @brief 检查左发是否失效
     * @return true 表示左发失效
     */
    bool isLeftEngineFailed() const { return _left_engine_failed; }

    /**
     * @brief 检查右发是否失效
     * @return true 表示右发失效
     */
    bool isRightEngineFailed() const { return _right_engine_failed; }

private:
    bool _is_active;
    bool _left_engine_failed;
    bool _right_engine_failed;
    float _time_since_failure;

    EngineFailurePhase _phase;

    float _rudder_trim;             ///< 方向舵配平（rad）
    float _aileron_trim;            ///< 副翼配平（rad）
    float _rudder_cmd;              ///< 方向舵指令（rad）

    float _target_speed;            ///< 单发目标速度（knots）
    float _target_thrust;           ///< 单发目标推力（N1 %）

    float _yaw_integral;            ///< 偏航积分项
    float _roll_integral;           ///< 滚转积分项

    /**
     * @struct Gains
     * @brief PID 控制器增益
     */
    struct Gains {
        // 方向舵增益
        float rudder_kp;
        float rudder_ki;
        float rudder_kd;

        // 副翼增益
        float aileron_kp;
        float aileron_ki;

        // 推力不对称补偿增益
        float asymmetry_kp;         ///< 推力不对称 → 方向舵映射系数
        float asymmetry_ki;         ///< 推力不对称积分

        // 配平速率限制
        float trim_rate_max;        ///< 最大配平速率 (deg/s)
    } _gains;

    float _last_yaw_error;
    float _last_asymmetry;

    /**
     * @brief 更新失效阶段
     */
    void updatePhase(const AircraftState& state);

    /**
     * @brief 计算推力不对称补偿
     * 
     * 根据两发动机推力差计算需要的方向舵配平。
     */
    float computeAsymmetryCompensation(const AircraftState& state, float dt);

    /**
     * @brief 计算航向补偿
     * 
     * 根据航向误差和偏航速率计算方向舵指令。
     */
    float computeYawCompensation(const AircraftState& state, float dt);

    /**
     * @brief 计算滚转补偿
     * 
     * 根据滚转角误差计算副翼配平。
     */
    float computeRollCompensation(const AircraftState& state, float dt);

    /**
     * @brief 计算单发目标速度
     * 
     * 单发爬升性能降低，需要调整目标速度。
     */
    float computeTargetSpeed(const AircraftState& state);

    /**
     * @brief 计算单发目标推力
     * 
     * 工作发动机需要增加推力以补偿失效发动机。
     */
    float computeTargetThrust(const AircraftState& state);

    /**
     * @brief 限制配平速率
     * 
     * 防止配平变化过快导致飞行员不适。
     */
    float limitTrimRate(float current, float target, float dt);

    /**
     * @brief 计算立即响应方向舵指令
     * 
     * 在失效瞬间快速踩舵抵消偏航趋势。
     */
    float computeImmediateRudder(const AircraftState& state);
};

} // namespace A320