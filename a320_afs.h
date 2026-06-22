/**
 * @file a320_afs.h
 * @brief A320 自动飞行系统（AutoFlightSystem）
 * 
 * AFSS 是自动驾驶系统的核心整合类，负责协调以下子系统：
 * - FlightControlComputer (FCC): 飞行控制计算机，处理姿态、航向、高度等控制
 * - Autothrottle: 自动油门系统
 * - FlightManagementSystem (FMS): 飞行管理系统
 * 
 * @author AutoGen
 * @date 2026
 */

#pragma once

#include "a320_autopilot_common.h"
#include "a320_fcc.h"
#include "a320_autothrottle.h"
#include "a320_fms.h"

namespace A320 {

/**
 * @class AutoFlightSystem
 * @brief 自动飞行系统
 * 
 * 负责协调各子系统的工作，提供统一的自动驾驶接口。
 * 主要功能包括：
 * - 启用/禁用自动驾驶和自动油门
 * - 设置飞行模式和目标参数
 * - 执行复飞程序
 * - 执行飞行包线保护
 */
class AutoFlightSystem {
public:
    /**
     * @brief 构造函数
     */
    AutoFlightSystem();

    /**
     * @brief 初始化系统
     * 
     * 初始化所有子系统，重置积分器和状态机。
     */
    void initialize();

    /**
     * @brief 启用自动驾驶
     */
    void enableAutopilot();

    /**
     * @brief 禁用自动驾驶
     * 
     * 禁用后自动切换到 OFF 模式。
     */
    void disableAutopilot();

    /**
     * @brief 获取自动驾驶启用状态
     * @return true 表示自动驾驶已启用
     */
    bool isAutopilotEnabled() const { return _ap_enabled; }

    /**
     * @brief 启用自动油门
     */
    void enableAutothrottle();

    /**
     * @brief 禁用自动油门
     */
    void disableAutothrottle();

    /**
     * @brief 获取自动油门启用状态
     * @return true 表示自动油门已启用
     */
    bool isAutothrottleEnabled() const { return _at_enabled; }

    /**
     * @brief 设置自动飞行模式
     * 
     * @param mode 目标模式（如 ALT_HOLD, HDG_HOLD, VS_HOLD 等）
     */
    void setAFMode(AFMode mode);

    /**
     * @brief 获取当前自动飞行模式
     * @return 当前 AFMode 值
     */
    AFMode getAFMode() const { return _fcc.getAFMode(); }

    /**
     * @brief 设置目标高度
     * @param altitude_ft 目标高度（英尺）
     */
    void setTargetAltitude(float altitude_ft);

    /**
     * @brief 设置目标垂直速度
     * @param vs_fpm 目标垂直速度（英尺/分钟）
     */
    void setTargetVerticalSpeed(float vs_fpm);

    /**
     * @brief 设置目标空速
     * @param speed_knots 目标空速（节）
     */
    void setTargetSpeed(float speed_knots);

    /**
     * @brief 设置目标马赫数
     * @param mach 目标马赫数
     */
    void setTargetMach(float mach);

    /**
     * @brief 设置目标航向
     * @param heading_deg 目标航向（度）
     */
    void setTargetHeading(float heading_deg);

    /**
     * @brief 激活复飞程序
     * 
     * 启动复飞状态机，自动设置 TO/GA 推力和抬头指令。
     * 
     * @param state 当前飞机状态
     */
    void activateGoAround(const AircraftState& state);

    /**
     * @brief 检查复飞是否激活
     * @return true 表示复飞程序正在执行
     */
    bool isGoAroundActive() const { return _fcc.isGoAroundActive(); }

    /**
     * @brief 获取复飞控制器引用
     * @return GoAroundController 引用
     */
    const GoAroundController& getGoAroundController() const { return _fcc.getGoAroundController(); }

    /**
     * @brief 更新自动驾驶系统
     * 
     * 根据当前飞机状态计算控制指令。
     * 
     * @param state 当前飞机状态
     */
    void update(const AircraftState& state);

    /**
     * @brief 获取当前控制指令
     * @return ControlCommand 结构体
     */
    ControlCommand getControlCommand() const { return _control_command; }

    /**
     * @brief 获取复飞目标速度
     * @return 目标速度（节）
     */
    float getGoAroundTargetSpeed() const;

    /**
     * @brief 获取复飞目标垂直速度
     * @return 目标垂直速度（英尺/分钟）
     */
    float getGoAroundTargetVS() const;

    /**
     * @brief 获取复飞阶段名称
     * @return 当前复飞阶段的字符串描述
     */
    const char* getGoAroundPhaseName() const;

    /**
     * @brief 检查复飞是否完成
     * @return true 表示复飞程序已完成，可切换到正常爬升模式
     */
    bool isGoAroundCompleted() const;

private:
    bool _ap_enabled;              ///< 自动驾驶启用标志
    bool _at_enabled;              ///< 自动油门启用标志

    FlightControlComputer _fcc;    ///< 飞行控制计算机
    Autothrottle _autothrottle;    ///< 自动油门系统
    FlightManagementSystem _fms;   ///< 飞行管理系统

    ControlCommand _control_command; ///< 当前控制指令

    /**
     * @brief 处理模式切换逻辑
     * 
     * 在复飞完成后自动切换到 V/S 爬升模式。
     */
    void handleModeTransitions(const AircraftState& state);

    /**
     * @brief 执行飞行包线保护
     * 
     * 包括迎角保护、超速保护、低速保护等。
     */
    void enforceFlightEnvelope(const AircraftState& state);
};

} // namespace A320