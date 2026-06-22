/**
 * @file a320_goaround.h
 * @brief A320 复飞控制器（GoAroundController）
 * 
 * 实现 A320 标准复飞程序，包含五个阶段：
 * 1. INITIAL_ROTATION: 初始抬头阶段（0-3秒）
 * 2. CLIMB_OUT_V2: 初始爬升，保持 V2+10 速度（离地 - 400ft）
 * 3. ACCELERATION: 加速阶段，准备收襟翼（400ft - 1500ft）
 * 4. CLIMB_OUT: 正常爬升（1500ft - 3000ft）
 * 5. TRANSITION_TO_CLIMB: 过渡到正常爬升模式（> 3000ft）
 * 
 * @author AutoGen
 * @date 2026
 */

#pragma once

#include "a320_autopilot_common.h"

namespace A320 {

/**
 * @class GoAroundController
 * @brief 复飞控制器
 * 
 * 实现完整的复飞状态机，负责：
 * - 阶段管理和状态转换
 * - 俯仰和滚转控制
 * - 速度和垂直速度目标管理
 * - 构型约束（襟翼/起落架收放时机）
 * - 迎角保护
 */
class GoAroundController {
public:
    /**
     * @brief 构造函数
     */
    GoAroundController();

    /**
     * @brief 激活复飞程序
     * 
     * 初始化复飞状态机，记录当前状态作为初始条件。
     * 
     * @param state 当前飞机状态
     */
    void activate(const AircraftState& state);

    /**
     * @brief 重置复飞控制器
     */
    void reset();

    /**
     * @brief 更新复飞控制器
     * 
     * 推进状态机，计算控制指令。
     * 
     * @param state 当前飞机状态
     * @param dt 时间步长（秒）
     */
    void update(const AircraftState& state, float dt);

    /**
     * @brief 获取当前复飞阶段
     * @return GoAroundPhase 枚举值
     */
    GoAroundPhase getPhase() const { return _phase; }

    /**
     * @brief 获取当前复飞阶段名称
     * @return 阶段名称字符串
     */
    const char* getPhaseName() const;

    /**
     * @brief 获取俯仰指令
     * @return 俯仰指令（弧度）
     */
    float getPitchCommand() const { return _pitch_cmd; }

    /**
     * @brief 获取滚转指令
     * @return 滚转指令（弧度）
     */
    float getRollCommand() const { return _roll_cmd; }

    /**
     * @brief 获取目标速度
     * @return 目标速度（节）
     */
    float getTargetSpeed() const { return _target_speed; }

    /**
     * @brief 获取目标垂直速度
     * @return 目标垂直速度（英尺/分钟）
     */
    float getTargetVerticalSpeed() const { return _target_vs; }

    /**
     * @brief 检查是否允许收襟翼
     * @return true 表示可以收襟翼
     */
    bool isFlapRetractionAllowed() const { return _allow_flap_retraction; }

    /**
     * @brief 检查是否允许收起落架
     * 
     * 只有在正爬升率建立后才允许收起落架。
     * 
     * @return true 表示可以收起落架
     */
    bool isGearRetractionAllowed() const { return _allow_gear_retraction && _positive_climb_established; }

    /**
     * @brief 检查复飞是否激活
     * @return true 表示复飞正在执行
     */
    bool isActive() const { return _is_active; }

    /**
     * @brief 检查复飞是否完成
     * @return true 表示复飞程序已完成
     */
    bool isCompleted() const { return _phase == GoAroundPhase::TRANSITION_TO_CLIMB; }

private:
    bool _is_active;                      ///< 复飞激活标志
    float _time_since_activation;         ///< 复飞激活后经过的时间（秒）

    GoAroundPhase _phase;                 ///< 当前复飞阶段
    float _initial_pitch_rate;            ///< 初始抬头速率（度/秒）
    float _target_vs;                     ///< 目标垂直速度（英尺/分钟）
    float _target_speed;                  ///< 目标空速（节）
    float _target_pitch;                  ///< 目标俯仰角（弧度）
    float _target_heading;                ///< 目标航向（度）
    float _initial_altitude_ft;           ///< 激活时的初始高度（英尺）
    bool _positive_climb_established;     ///< 正爬升率建立标志

    float _pitch_cmd;                     ///< 当前俯仰指令（弧度）
    float _roll_cmd;                      ///< 当前滚转指令（弧度）

    bool _allow_flap_retraction;          ///< 允许收襟翼标志
    bool _allow_gear_retraction;          ///< 允许收起落架标志

    /**
     * @struct PitchController
     * @brief 俯仰 PID 控制器
     */
    struct PitchController {
        float kp;          ///< 比例增益
        float ki;          ///< 积分增益
        float kd;          ///< 微分增益
        float integral;    ///< 积分项
        float last_error;  ///< 上一次误差
    } _pitch_controller;

    /**
     * @struct RollController
     * @brief 滚转 PID 控制器
     */
    struct RollController {
        float kp;          ///< 比例增益
        float ki;          ///< 积分增益
        float kd;          ///< 微分增益
        float integral;    ///< 积分项
        float last_error;  ///< 上一次误差
    } _roll_controller;

    /**
     * @brief 更新复飞阶段
     * 
     * 根据高度和时间判断阶段转换。
     */
    void updatePhase(const AircraftState& state);

    /**
     * @brief 计算俯仰指令
     * 
     * 使用 PID 控制器计算俯仰指令。
     */
    float computePitch(const AircraftState& state, float dt);

    /**
     * @brief 计算滚转指令
     * 
     * 使用 PID 控制器计算滚转指令（航向保持）。
     */
    float computeRoll(const AircraftState& state, float dt);

    /**
     * @brief 计算目标俯仰角
     * 
     * 根据当前阶段和状态计算目标俯仰角。
     */
    float computeTargetPitch(const AircraftState& state);

    /**
     * @brief 根据速度误差计算俯仰修正
     * @param speed_error_knots 速度误差（节）
     * @return 俯仰修正量（弧度）
     */
    float computePitchForSpeed(float speed_error_knots);

    /**
     * @brief 根据高度误差计算俯仰修正
     * @param altitude_error_ft 高度误差（英尺）
     * @return 俯仰修正量（弧度）
     */
    float computePitchForAltitude(float altitude_error_ft);

    /**
     * @brief 根据垂直速度误差计算俯仰修正
     * @param vs_error_fpm 垂直速度误差（英尺/分钟）
     * @return 俯仰修正量（弧度）
     */
    float computePitchForVS(float vs_error_fpm);

    /**
     * @brief 混合俯仰指令
     * 
     * 根据速度误差动态调整垂直速度和速度控制的权重。
     */
    float blendPitchCommands(float pitch_for_vs, float pitch_for_speed, float speed_error);

    /**
     * @brief 限制复飞阶段的俯仰角
     * 
     * 考虑迎角保护、速度保护等约束。
     */
    float limitPitchForGoAround(float pitch, const AircraftState& state);

    /**
     * @brief 限制俯仰角速度
     * @param pitch_rate 俯仰角速度
     * @param max_rate_deg_s 最大角速度（度/秒）
     * @return 限制后的角速度
     */
    float limitPitchRate(float pitch_rate, float max_rate_deg_s = 5.0f);
};

} // namespace A320