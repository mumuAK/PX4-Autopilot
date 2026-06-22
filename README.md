# A320 自动驾驶系统

A320 自动驾驶系统是一个基于 C++ 的固定翼飞行仿真自动驾驶模块，实现了空客 A320 的核心自动驾驶功能。

## 功能特性

- **姿态保持 (ATT_HOLD)** - 保持当前俯仰姿态
- **航向保持 (HDG_HOLD)** - 保持目标航向
- **高度保持 (ALT_HOLD)** - 保持目标高度
- **垂直速度控制 (VS_HOLD)** - 保持目标爬升率/下降率
- **速度保持 (SPD_HOLD)** - 保持目标空速
- **复飞 (GO_AROUND)** - 完整的复飞状态机

## 架构设计

```
┌─────────────────────────────────────────────────────────────┐
│                      test_driver (主控类)                   │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐   │
│  │ map_in   │  │  step    │  │ map_out  │  │execute_cmd│  │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘   │
└───────┼─────────────┼─────────────┼─────────────┼─────────┘
        │             │             │             │
        ▼             ▼             ▼             ▼
┌─────────────────────────────────────────────────────────────┐
│                  AutoFlightSystem (AFS)                    │
│  ┌─────────────────┐  ┌─────────────────┐  ┌───────────┐   │
│  │FlightControlComp│  │  Autothrottle  │  │  FMS     │   │
│  │     (FCC)       │  │                 │  │           │   │
│  └────────┬────────┘  └────────┬────────┘  └─────┬─────┘   │
│           │                    │                  │         │
│           ▼                    ▼                  ▼         │
│  ┌─────────────────────────────────────────────────────┐   │
│  │         GoAroundController (复飞状态机)             │   │
│  │  INITIAL_ROTATION → CLIMB_OUT_V2 → ACCELERATION   │   │
│  │  → CLIMB_OUT → TRANSITION_TO_CLIMB                │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

## 文件结构

| 文件 | 描述 |
|------|------|
| `a320_autopilot_common.h` | 公共类型定义（AircraftState, ControlCommand 等） |
| `a320_autothrottle.h/cpp` | 自动油门控制器 |
| `a320_goaround.h/cpp` | 复飞控制器 |
| `a320_fcc.h/cpp` | 飞行控制计算机 |
| `a320_fms.h/cpp` | 飞行管理系统 |
| `a320_afs.h/cpp` | 自动飞行系统（整合模块） |
| `test_driver.h/cpp` | 主控类（外部接口） |
| `test_driver_demo.cpp` | 演示程序 |

## 快速开始

### 编译

```bash
g++ -std=c++17 -Wall -Wextra -I/usr/include/eigen3 -I. \
    a320_autothrottle.cpp a320_goaround.cpp a320_fcc.cpp \
    a320_fms.cpp a320_afs.cpp test_driver.cpp test_driver_demo.cpp \
    -o a320_autopilot_demo
```

### 运行演示

```bash
./a320_autopilot_demo
```

## API 使用

### 基本流程

```cpp
#include "test_driver.h"

// 创建主控类
test_driver driver;
driver.init();

// 设置初始状态
AircraftState state = {...};

// 启用自动驾驶和自动油门
driver.execute_command("AP_ON");
driver.execute_command("AT_ON");

// 设置飞行模式和目标参数
driver.execute_command("SET_ALT", 5000.0f);
driver.execute_command("ALT_HOLD");

// 主循环
while (running) {
    // 输入当前状态
    driver.map_in(state);
    
    // 执行控制计算
    driver.step(0.05f);
    
    // 获取控制指令
    ControlCommand cmd;
    driver.map_out(cmd);
    
    // 应用控制指令到飞机模型
    applyControl(cmd);
}
```

### 支持的命令

| 命令 | 参数 | 功能 |
|------|------|------|
| `AP_ON` | - | 启用自动驾驶 |
| `AP_OFF` | - | 禁用自动驾驶 |
| `AT_ON` | - | 启用自动油门 |
| `AT_OFF` | - | 禁用自动油门 |
| `HDG_HOLD` | - | 航向保持模式 |
| `ALT_HOLD` | - | 高度保持模式 |
| `VS_HOLD` | - | 垂直速度模式 |
| `SPD_HOLD` | - | 速度保持模式 |
| `ATT_HOLD` | - | 姿态保持模式 |
| `GO_AROUND` | - | 复飞模式 |
| `CLIMB` | - | 爬升模式 |
| `DESCENT` | - | 下降模式 |
| `SET_HDG` | 角度(°) | 设置目标航向 |
| `SET_ALT` | 高度(ft) | 设置目标高度 |
| `SET_VS` | 爬升率(fpm) | 设置目标垂直速度 |
| `SET_SPD` | 速度(knots) | 设置目标速度 |
| `SET_MACH` | 马赫数 | 设置目标马赫 |

## 核心数据结构

### AircraftState

飞机状态结构体，包含位置、速度、姿态、发动机参数等：

```cpp
struct AircraftState {
    double lat, lon;           // 经纬度
    float alt_msl, alt_std;    // 高度
    float tas, cas, mach;      // 速度
    float roll, pitch, yaw;    // 姿态角
    float n1_left, n1_right;   // 发动机转速
    float aoa, alpha_prot;     // 迎角和迎角保护
    // ...
};
```

### ControlCommand

控制指令结构体：

```cpp
struct ControlCommand {
    float roll_cmd;      // 滚转角指令 (rad)
    float pitch_cmd;     // 俯仰角指令 (rad)
    float yaw_cmd;       // 航向角指令 (rad)
    float throttle_cmd;  // 油门指令 (0-100%)
    float flap_cmd;      // 襟翼指令
    float gear_cmd;      // 起落架指令
};
```

## 复飞状态机

复飞程序包含五个阶段：

| 阶段 | 条件 | 特征 |
|------|------|------|
| INITIAL_ROTATION | 0-3秒 | 快速抬头至15° |
| CLIMB_OUT_V2 | < 400ft | 保持 V2+10 速度 |
| ACCELERATION | 400-1500ft | 允许收襟翼，加速 |
| CLIMB_OUT | 1500-3000ft | 允许收起落架 |
| TRANSITION_TO_CLIMB | > 3000ft | 完成复飞，切换到正常爬升 |

## 飞行包线保护

系统实现了以下保护功能：

- **迎角保护** - 接近 alpha_prot 时自动推杆
- **超速保护** - 马赫数 > 0.86 时减小油门
- **低速保护** - 速度 < 140kts 时增加油门

## 依赖

- C++17 或更高版本
- Eigen3（可选，当前代码未实际使用）

## 许可证

MIT License

## 作者

AutoGen