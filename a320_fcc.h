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
 * - 地面轨迹跟踪（GROUND_TRACK）
 * - 单发失效补偿（ENGINE_OUT）
 * 
 * @author AutoGen
 * @date 2026
 */

#pragma once

#include "a320_autopilot_common.h"
#include "a320_goaround.h"
#include "a320_ground_control.h"
#include "a320_engine_out.h"

namespace A320 {

/**
 * @class FlightControlComputer
 * @brief 飞行控制计算机
 * 
 * 负责计算俯仰、滚转、方向舵、前轮转弯等控制指令。
 * 根据飞行状态（空中/地面）和发动机状态自动选择控制策略。
 */
class FlightControlComputer {
public:
    /**
     * @brief 构造函数
     */
    FlightControlComputer();

    /**
     * @brief 初始化 FCC
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
     * @brief 设置跑道航向（地面控制用）
     * @param runway_heading_deg 跑道航向（度）
     */
    void setRunwayHeading(float runway_heading_deg);

    /**
     * @brief 激活复飞程序
     * @param state 当前飞机状态
     */
    void activateGoAround(const AircraftState& state);

    /**
     * @brief 激活单发失效模式
     * @param state 当前飞机状态
     * @param failed_side 失效发动机侧（"LEFT" 或 "RIGHT"）
     */
    void activateEngineOut(const AircraftState& state, const std::string& failed_side);

    /**
     * @brief 检查复飞是否激活
     * @return true 表示复飞正在执行
     */
    bool isGoAroundActive() const { return _go_around.isActive(); }

    /**
     * @brief 检查单发失效是否激活
     * @return true 表示单发补偿正在执行
     */
    bool isEngineOutActive() const { return _engine_out.isActive(); }

    /**
     * @brief 获取复飞控制器
     * @return GoAroundController 引用
     */
    const GoAroundController& getGoAroundController() const { return _go_around; }

    /**
     * @brief 获取地面控制器
     * @return GroundController 引用
     */
    const GroundController& getGroundController() const { return _ground_control; }

    /**
     * @brief 获取单发失效控制器
     * @return EngineOutController 引用
     */
    const EngineOutController& getEngineOutController() const { return _engine_out; }

    /**
     * @brief 计算控制指令
     * 
     * 根据当前飞机状态和飞行模式，计算所有控制指令。
     * 自动判断空中/地面状态并选择控制策略。
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
    float _runway_heading;          ///< 跑道航向（度）

    float _pitch_integral;          ///< 俯仰积分项
    float _vs_integral;             ///< 垂直速度积分项
    float _speed_integral;          ///< 速度积分项

    GoAroundController _go_around;  ///< 复飞控制器
    GroundController _ground_control; ///< 地面控制器
    EngineOutController _engine_out; ///< 单发失效控制器

    /**
     * @struct Gains
     * @brief PID 控制器增益参数
     */
    struct Gains {
        float pitch_kp;
        float pitch_ki;
        float pitch_kd;
        float roll_kp;
        float roll_ki;
        float roll_kd;
        float vs_kp;
        float vs_ki;
        float alt_kp;
        float alt_ki;
        float speed_kp;
        float speed_ki;
    } _gains;

    /**
     * @brief 限制俯仰角范围
     */
    float limitPitch(float pitch_rad) const;

    /**
     * @brief 限制滚转角范围
     */
    float limitRoll(float roll_rad) const;

    /**
     * @brief 高度保持控制
     */
    float altitudeHoldControl(const AircraftState& state);

    /**
     * @brief 垂直速度控制
     */
    float verticalSpeedControl(const AircraftState& state);

    /**
     * @brief 速度保持控制
     */
    float speedHoldControl(const AircraftState& state);

    /**
     * @brief 姿态保持控制
     */
    float attitudeHoldControl(const AircraftState& state);

    /**
     * @brief 航向保持控制（空中）
     */
    float headingHoldControl(const AircraftState& state);

    /**
     * @brief 复飞控制
     */
    float goAroundControl(const AircraftState& state);

    /**
     * @brief 地面轨迹控制
     */
    void groundTrackControl(const AircraftState& state, ControlCommand& cmd, float dt);

    /**
     * @brief 单发失效控制
     */
    void engineOutControl(const AircraftState& state, ControlCommand& cmd, float dt);

    /**
     * @brief 判断是否需要地面控制
     */
    bool isGroundControlNeeded(const AircraftState& state) const;

    /**
     * @brief 判断是否需要单发补偿
     */
    bool isEngineOutCompensationNeeded(const AircraftState& state) const;
};

} // namespace A320