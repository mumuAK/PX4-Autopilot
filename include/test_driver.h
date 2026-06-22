/**
 * @file test_driver.h
 * @brief A320 自动驾驶系统主控类
 * 
 * test_driver 是 A320 自动驾驶系统的顶层接口类，提供简洁的接口供外部调用。
 * 主要功能包括：
 * - 初始化自动驾驶系统
 * - 接收飞机状态输入（map_in）
 * - 执行控制计算（step）
 * - 输出控制指令（map_out）
 * - 接收外部命令（execute_command）
 * 
 * @author AutoGen
 * @date 2026
 */

#pragma once

#include "a320_autopilot_common.h"
#include "a320_afs.h"

namespace A320 {

/**
 * @class test_driver
 * @brief 自动驾驶系统主控类
 * 
 * 提供标准化的输入输出接口，将复杂的自动驾驶逻辑封装在内部。
 * 外部系统只需调用以下四个核心方法：
 * 1. init() - 初始化
 * 2. map_in() - 输入状态
 * 3. step() - 执行计算
 * 4. map_out() - 获取指令
 * 
 * 同时支持通过 execute_command() 接收外部控制命令。
 */
class test_driver {
public:
    /**
     * @brief 构造函数
     */
    test_driver();

    /**
     * @brief 初始化自动驾驶系统
     * 
     * 初始化所有子系统（FCC、Autothrottle、FMS），设置初始状态。
     * 必须在使用其他方法之前调用。
     */
    void init();

    /**
     * @brief 输入当前飞机状态
     * 
     * 将外部飞机状态传入自动驾驶系统，供后续计算使用。
     * 
     * @param state 当前飞机状态结构体
     */
    void map_in(const AircraftState& state);

    /**
     * @brief 执行一步控制计算
     * 
     * 根据当前输入的飞机状态，执行自动驾驶控制计算。
     * 计算结果存储在内部，通过 map_out() 获取。
     * 
     * @param dt 时间步长（秒），默认 0.05s（20Hz）
     */
    void step(float dt = 0.05f);

    /**
     * @brief 输出控制指令
     * 
     * 将计算得到的控制指令输出到外部系统。
     * 
     * @param cmd [out] 控制指令结构体，包含俯仰、滚转、油门等指令
     */
    void map_out(ControlCommand& cmd);

    /**
     * @brief 执行外部命令
     * 
     * 接收并执行外部发来的控制命令，如启用/禁用自动驾驶、设置目标参数等。
     * 
     * @param command 命令名称（如 "AP_ON", "ALT_HOLD", "SET_ALT"）
     * @param value 命令参数（如目标高度、速度等，默认为 0.0）
     */
    void execute_command(const std::string& command, float value = 0.0f);

    /**
     * @brief 获取自动驾驶启用状态
     * @return true 表示自动驾驶已启用
     */
    bool is_autopilot_enabled() const { return _afs.isAutopilotEnabled(); }

    /**
     * @brief 获取自动油门启用状态
     * @return true 表示自动油门已启用
     */
    bool is_autothrottle_enabled() const { return _afs.isAutothrottleEnabled(); }

    /**
     * @brief 获取当前自动驾驶模式
     * @return 当前 AFMode 枚举值
     */
    AFMode get_current_mode() const { return _afs.getAFMode(); }

private:
    AutoFlightSystem _afs;       ///< 自动飞行系统实例
    AircraftState _current_state; ///< 当前飞机状态缓存
    ControlCommand _current_command; ///< 当前计算的控制指令
};

} // namespace A320