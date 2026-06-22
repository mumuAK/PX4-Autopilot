# A320 自动驾驶系统

基于 C++ 的 A320 固定翼飞行仿真自动驾驶模块。

## 功能特性

- **自动油门 (Autothrottle)**：速度保持、马赫保持
- **飞行控制计算机 (FCC)**：姿态保持、航向保持、高度保持、垂直速度控制
- **复飞控制器 (Go-Around)**：五阶段复飞程序
- **地面控制器 (Ground Control)**：前轮转弯、方向舵、刹车控制
- **单发失效补偿 (Engine Out)**：方向舵配平补偿

## 目录结构

```
/
├── include/              # 头文件
│   ├── a320_autopilot_common.h    # 公共类型定义
│   ├── a320_autothrottle.h        # 自动油门
│   ├── a320_goaround.h            # 复飞控制器
│   ├── a320_ground_control.h      # 地面控制器
│   ├── a320_engine_out.h          # 单发失效补偿
│   ├── a320_fcc.h                 # 飞行控制计算机
│   ├── a320_fms.h                 # 飞行管理系统
│   ├── a320_afs.h                 # 自动飞行系统
│   └── test_driver.h              # 主控类
├── src/
│   └── autopilot/        # 源代码
│       ├── a320_autothrottle.cpp
│       ├── a320_goaround.cpp
│       ├── a320_ground_control.cpp
│       ├── a320_engine_out.cpp
│       ├── a320_fcc.cpp
│       ├── a320_fms.cpp
│       ├── a320_afs.cpp
│       └── test_driver.cpp
├── examples/             # 示例程序
│   ├── a320_autopilot_demo.cpp
│   └── test_driver_demo.cpp
├── tests/                # 测试文件
└── README.md
```

## 快速开始

### 编译

```bash
g++ -std=c++17 -Iinclude -o bin/a320_autopilot src/autopilot/*.cpp examples/test_driver_demo.cpp
```

### 运行

```bash
./bin/a320_autopilot
```

## API 使用

```cpp
#include "include/test_driver.h"

A320::test_driver driver;
driver.init();

// 设置目标
driver.execute_command("SET_ALT", 5000.0f);
driver.execute_command("SET_SPD", 250.0f);

// 启用模式
driver.execute_command("AP_ON");
driver.execute_command("ALT_HOLD");
driver.execute_command("AT_ON");

// 主循环
while (running) {
    driver.map_in(current_state);
    driver.step(0.05f);
    
    A320::ControlCommand cmd;
    driver.map_out(cmd);
    
    // 应用控制指令
}
```

## 支持的命令

| 命令 | 参数 | 功能 |
|------|------|------|
| AP_ON | - | 启用自动驾驶 |
| AP_OFF | - | 禁用自动驾驶 |
| AT_ON | - | 启用自动油门 |
| AT_OFF | - | 禁用自动油门 |
| HDG_HOLD | - | 航向保持模式 |
| ALT_HOLD | - | 高度保持模式 |
| VS_HOLD | - | 垂直速度模式 |
| SPD_HOLD | - | 速度保持模式 |
| ATT_HOLD | - | 姿态保持模式 |
| GO_AROUND | - | 复飞模式 |
| CLIMB | - | 爬升模式 |
| DESCENT | - | 下降模式 |
| GROUND_TRACK | - | 地面轨迹跟踪 |
| ENGINE_OUT_LEFT | - | 左发失效 |
| ENGINE_OUT_RIGHT | - | 右发失效 |
| SET_HDG | 角度(°) | 设置目标航向 |
| SET_ALT | 高度(ft) | 设置目标高度 |
| SET_VS | 爬升率(fpm) | 设置目标垂直速度 |
| SET_SPD | 速度(knots) | 设置目标速度 |
| SET_RWY_HDG | 角度(°) | 设置跑道航向 |

## 地面控制逻辑

| 速度 | 控制方式 |
|------|----------|
| < 30 kts | 前轮转弯 (NWS) |
| >= 30 kts | 方向舵 (脚蹬) |
| 速度保持 | 刹车 |

## 单发失效补偿

- 方向舵配平补偿推力不对称
- 俯仰、横滚、油门保持原有控制

## 许可证

MIT License