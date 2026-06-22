/**
 * @file a320_autothrottle.h
 * @brief A320 自动油门系统（Autothrottle）
 * 
 * 自动油门系统负责控制发动机推力，支持多种工作模式：
 * - 速度保持模式（低速时）
 * - 马赫保持模式（高速时）
 * - 推力保持模式（爬升时）
 * - 复飞模式（TO/GA 推力）
 * 
 * @author AutoGen
 * @date 2026
 */

#pragma once

#include "a320_autopilot_common.h"

namespace A320 {

/**
 * @class Autothrottle
 * @brief 自动油门控制器
 * 
 * 负责计算油门指令，根据飞行阶段和目标参数自动调整发动机推力。
 * 使用 PID 控制器实现精确的速度/推力控制。
 */
class Autothrottle {
public:
    /**
     * @brief 构造函数
     */
    Autothrottle();

    /**
     * @brief 设置自动油门模式
     * @param enabled true 启用，false 禁用
     */
    void setMode(bool enabled);

    /**
     * @brief 获取自动油门状态
     * @return true 表示自动油门已启用
     */
    bool isEnabled() const { return _enabled; }

    /**
     * @brief 设置目标速度
     * @param speed_knots 目标空速（节）
     */
    void setTargetSpeed(float speed_knots);

    /**
     * @brief 设置目标马赫数
     * @param mach 目标马赫数
     */
    void setTargetMach(float mach);

    /**
     * @brief 设置目标推力
     * @param n1_percent 目标 N1 转速百分比
     */
    void setTargetThrust(float n1_percent);

    /**
     * @brief 设置目标 EPR（发动机压力比）
     * @param epr 目标 EPR
     */
    void setTargetEPR(float epr);

    /**
     * @brief 设置飞行阶段
     * 
     * 不同飞行阶段使用不同的控制策略。
     * 
     * @param phase 飞行阶段
     */
    void setFlightPhase(FlightPhase phase);

    /**
     * @brief 计算油门指令
     * 
     * 根据当前飞机状态和目标参数计算油门指令。
     * 
     * @param state 当前飞机状态
     * @return 油门指令（0-100%）
     */
    float computeThrottle(const AircraftState& state);

private:
    bool _enabled;               ///< 自动油门启用标志
    float _target_speed;         ///< 目标速度（节）
    float _target_mach;          ///< 目标马赫数
    float _target_thrust;        ///< 目标推力（N1 %）
    float _target_epr;           ///< 目标 EPR

    FlightPhase _flight_phase;   ///< 当前飞行阶段

    /**
     * @struct PID
     * @brief PID 控制器结构体
     */
    struct PID {
        float kp;             ///< 比例增益
        float ki;             ///< 积分增益
        float kd;             ///< 微分增益
        float integral;       ///< 积分项
        float max_integral;   ///< 积分上限
        float min_output;     ///< 输出下限
        float max_output;     ///< 输出上限
    };

    PID _speed_pid;          ///< 速度控制 PID
    PID _mach_pid;           ///< 马赫控制 PID
    PID _thrust_pid;         ///< 推力控制 PID

    float _last_speed_error; ///< 上一次速度误差
    float _last_time;        ///< 上一次更新时间

    /**
     * @brief 速度控制
     * @param state 当前飞机状态
     * @return 油门指令
     */
    float computeSpeedControl(const AircraftState& state);

    /**
     * @brief 马赫控制
     * @param state 当前飞机状态
     * @return 油门指令
     */
    float computeMachControl(const AircraftState& state);

    /**
     * @brief 推力控制
     * @param state 当前飞机状态
     * @return 油门指令
     */
    float computeThrustControl(const AircraftState& state);
};

} // namespace A320