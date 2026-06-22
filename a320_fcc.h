/**
 * @file a320_fcc.h
 * @brief A320 飞行控制计算机（FlightControlComputer）
 * 
 * FCC 是自动驾驶系统的核心控制单元，负责：
 * - 姿态保持（ATT_HOLD）
 * - 航向保持（HDG_HOLD）
 * - 高度保持（ALT_HOLD）
 * - 垂直速度控制（VERTICAL_SPEED）
 * - 速度保持（SPEED_HOLD）
 * - 复飞控制（GO_AROUND）
 * 
 * @author AutoGen
 * @date 2026
 */

#pragma once

#include "a320_autopilot_common.h"
#include "a320_goaround.h"

namespace A320 {

/**
 * @class FlightControlComputer
 * @brief 飞行控制计算机
 * 
 * 负责计算俯仰和滚转控制指令，支持多种自动飞行模式。
 * 使用 PID 控制器实现精确的姿态和轨迹控制。
 */
class FlightControlComputer {
public:
    /**
     * @brief 构造函数
     */
    FlightControlComputer();

    /**
     * @brief 初始化 FCC
     * 
     * 重置积分器状态和复飞控制器。
     */
    void initialize();

    /**
     * @brief 设置自动飞行模式
     * @param mode 目标模式
     */
    void setAFMode(AFMode mode);

    /**
     * @brief 获取当前自动飞行模式
     * @return 当前 AFMode 值
     */
    AFMode getAFMode() const { return _current_mode; }

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
     * @param state 当前飞机状态
     */
    void activateGoAround(const AircraftState& state);

    /**
     * @brief 检查复飞是否激活
     * @return true 表示复飞正在执行
     */
    bool isGoAroundActive() const { return _go_around.isActive(); }

    /**
     * @brief 获取复飞控制器
     * @return GoAroundController 引用
     */
    const GoAroundController& getGoAroundController() const { return _go_around; }

    /**
     * @brief 计算控制指令
     * 
     * 根据当前飞机状态和飞行模式，计算俯仰和滚转指令。
     * 
     * @param state 当前飞机状态
     * @return ControlCommand 控制指令
     */
    ControlCommand computeControl(const AircraftState& state);

private:
    AFMode _current_mode;           ///< 当前自动飞行模式

    float _target_altitude;         ///< 目标高度（英尺）
    float _target_vs;               ///< 目标垂直速度（英尺/分钟）
    float _target_speed;            ///< 目标空速（节）
    float _target_mach;             ///< 目标马赫数
    float _target_heading;          ///< 目标航向（度）
    float _target_pitch;            ///< 目标俯仰角（弧度）

    float _pitch_integral;          ///< 俯仰积分项
    float _vs_integral;             ///< 垂直速度积分项
    float _speed_integral;          ///< 速度积分项

    GoAroundController _go_around;  ///< 复飞控制器

    /**
     * @struct Gains
     * @brief PID 控制器增益参数
     */
    struct Gains {
        float pitch_kp;   ///< 俯仰比例增益
        float pitch_ki;   ///< 俯仰积分增益
        float pitch_kd;   ///< 俯仰微分增益
        float roll_kp;    ///< 滚转比例增益
        float roll_ki;    ///< 滚转积分增益
        float roll_kd;    ///< 滚转微分增益
        float vs_kp;      ///< 垂直速度比例增益
        float vs_ki;      ///< 垂直速度积分增益
        float alt_kp;     ///< 高度比例增益
        float alt_ki;     ///< 高度积分增益
        float speed_kp;   ///< 速度比例增益
        float speed_ki;   ///< 速度积分增益
    } _gains;

    /**
     * @brief 限制俯仰角范围
     * @param pitch_rad 俯仰角（弧度）
     * @return 限制后的俯仰角
     */
    float limitPitch(float pitch_rad) const;

    /**
     * @brief 限制滚转角范围
     * @param roll_rad 滚转角（弧度）
     * @return 限制后的滚转角
     */
    float limitRoll(float roll_rad) const;

    /**
     * @brief 高度保持控制
     * @param state 当前飞机状态
     * @return 俯仰指令（弧度）
     */
    float altitudeHoldControl(const AircraftState& state);

    /**
     * @brief 垂直速度控制
     * @param state 当前飞机状态
     * @return 俯仰指令（弧度）
     */
    float verticalSpeedControl(const AircraftState& state);

    /**
     * @brief 速度保持控制
     * @param state 当前飞机状态
     * @return 俯仰指令（弧度）
     */
    float speedHoldControl(const AircraftState& state);

    /**
     * @brief 姿态保持控制
     * @param state 当前飞机状态
     * @return 俯仰指令（弧度）
     */
    float attitudeHoldControl(const AircraftState& state);

    /**
     * @brief 航向保持控制
     * @param state 当前飞机状态
     * @return 滚转指令（弧度）
     */
    float headingHoldControl(const AircraftState& state);

    /**
     * @brief 复飞控制
     * @param state 当前飞机状态
     * @return 俯仰指令（弧度）
     */
    float goAroundControl(const AircraftState& state);
};

} // namespace A320