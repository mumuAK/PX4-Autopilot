/**
 * @file a320_engine_out.h
 * @brief A320 单发失效脚蹬补偿控制器（简化版）
 * 
 * 简化设计：仅模拟飞行操纵，补偿单发偏航。
 * - 俯仰控制：原有俯仰控制器
 * - 横滚控制：原有横滚控制器
 * - 自动油门：正常工作
 * - 脚蹬补偿：计算方向舵配平抵消偏航力矩
 * 
 * @author AutoGen
 * @date 2026
 */

#pragma once

#include "a320_autopilot_common.h"

namespace A320 {

/**
 * @class EngineOutController
 * @brief 单发失效脚蹬补偿控制器
 * 
 * 负责计算单发失效时的方向舵配平指令，补偿推力不对称导致的偏航。
 * 其他控制（俯仰、横滚、油门）由原有控制器处理。
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
     * @param state 当前飞机状态
     * @param failed_side 失效侧（"LEFT" 或 "RIGHT"）
     */
    void activate(const AircraftState& state, const std::string& failed_side);

    /**
     * @brief 重置控制器
     */
    void reset();

    /**
     * @brief 更新控制器
     * @param state 当前飞机状态
     * @param dt 时间步长（秒）
     */
    void update(const AircraftState& state, float dt);

    /**
     * @brief 获取方向舵配平指令
     * @return 方向舵配平（rad）
     */
    float getRudderTrim() const { return _rudder_trim; }

    /**
     * @brief 获取方向舵指令（立即响应）
     * @return 方向舵指令（rad）
     */
    float getRudderCommand() const { return _rudder_cmd; }

    /**
     * @brief 检查是否激活
     * @return true 表示单发补偿正在执行
     */
    bool isActive() const { return _is_active; }

    /**
     * @brief 检查左发是否失效
     */
    bool isLeftEngineFailed() const { return _left_engine_failed; }

    /**
     * @brief 检查右发是否失效
     */
    bool isRightEngineFailed() const { return _right_engine_failed; }

    /**
     * @brief 获取失效阶段名称
     * @return 阶段名称
     */
    const char* getPhaseName() const;

private:
    bool _is_active;
    bool _left_engine_failed;
    bool _right_engine_failed;

    float _rudder_trim;         ///< 方向舵配平
    float _rudder_cmd;          ///< 方向舵指令

    float _trim_rate_max;       ///< 最大配平速率 (rad/s)

    float computeRudderTrim(const AircraftState& state, float dt);
    float computeImmediateRudder(const AircraftState& state);
};

} // namespace A320