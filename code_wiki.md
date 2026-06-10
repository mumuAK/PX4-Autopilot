# PX4 Autopilot Code Wiki

## 1. 项目概述

PX4 Autopilot 是一个开源的无人机飞控系统，由 Dronecode Foundation（Linux Foundation 的一部分）维护。该项目支持多种飞行器平台，包括多旋翼、固定翼、VTOL、地面车等。

### 1.1 项目特性

- **模块化架构**：基于 uORB（微型对象请求代理）发布/订阅中间件，模块完全并行化且线程安全
- **广泛的硬件支持**：支持多种飞控板和传感器
- **开发者友好**：一流的 MAVLink 和 ROS 2/DDS 集成支持
- **多平台支持**：运行在 NuttX、Linux 和 macOS 上

### 1.2 支持的飞行器类型

| 类型 | 说明 |
|------|------|
| Multicopter | 多旋翼无人机 |
| Fixed Wing | 固定翼飞机 |
| VTOL | 垂直起降飞行器 |
| Rover | 地面车 |
| Airship | 飞艇 |
| Helicopter | 直升机（实验性） |

---

## 2. 项目架构

### 2.1 整体架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                        PX4 Autopilot                           │
├─────────────────────────────────────────────────────────────────┤
│                      Application Layer                          │
│  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────────────────┐   │
│  │ Commander│ │ Navigator│ │  MAVLink │ │     Events         │   │
│  └────┬────┘ └────┬────┘ └────┬────┘ └─────────────────────┘   │
├─────────────────────────────────────────────────────────────────┤
│                      Control Layer                              │
│  ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐   │
│  │ mc_att_control  │ │ mc_pos_control  │ │   mc_rate_ctrl  │   │
│  │ (多旋翼姿态)     │ │ (多旋翼位置)     │ │   (多旋翼速率)   │   │
│  ├─────────────────┤ ├─────────────────┤ ├─────────────────┤   │
│  │ fw_att_control  │ │ fw_lat_long    │ │   fw_rate_ctrl  │   │
│  │ (固定翼姿态)     │ │ (固定翼横向纵向) │ │   (固定翼速率)   │   │
│  ├─────────────────┤ ├─────────────────┤ ├─────────────────┤   │
│  │ vtol_att_control│ │ rover_control  │ │    gimbal       │   │
│  │   (VTOL姿态)     │ │   (地面车控制)   │ │   (云台控制)    │   │
│  └─────────────────┘ └─────────────────┘ └─────────────────┘   │
├─────────────────────────────────────────────────────────────────┤
│                      Estimation Layer                           │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │                      EKF2                               │   │
│  │  - 姿态估计 (Attitude)                                   │   │
│  │  - 位置估计 (Position)                                   │   │
│  │  - 速度估计 (Velocity)                                   │   │
│  │  - 风估计 (Wind)                                         │   │
│  │  - 地形估计 (Terrain)                                    │   │
│  └─────────────────────────────────────────────────────────┘   │
├─────────────────────────────────────────────────────────────────┤
│                      Sensor Layer                               │
│  ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐   │
│  │  Vehicle IMU    │ │   GPS Position  │ │  Magnetometer   │   │
│  ├─────────────────┤ ├─────────────────┤ ├─────────────────┤   │
│  │  Barometer      │ │  Optical Flow   │ │   Airspeed      │   │
│  ├─────────────────┤ ├─────────────────┤ ├─────────────────┤   │
│  │  Distance Sensor│ │     RC Input    │ │    Battery      │   │
│  └─────────────────┘ └─────────────────┘ └─────────────────┘   │
├─────────────────────────────────────────────────────────────────┤
│                      Platform Layer                             │
│  ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐   │
│  │     NuttX      │ │      POSIX      │ │      QURT      │   │
│  └─────────────────┘ └─────────────────┘ └─────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 目录结构

| 目录 | 职责 | 关键文件 |
|------|------|----------|
| `src/modules/` | 核心功能模块 | commander、ekf2、mavlink、sensors、mc_*、fw_* |
| `src/lib/` | 基础库 | mathlib、matrix、controllib、parameters、events |
| `msg/` | uORB 消息定义 | *.msg 文件 |
| `platforms/` | 平台适配层 | nuttx、posix、qurt、ros2 |
| `ROMFS/` | 只读文件系统 | 配置文件、参数 |
| `Tools/` | 开发工具 | 脚本、辅助工具 |
| `cmake/` | CMake 配置 | 构建系统配置 |

---

## 3. 核心模块说明

### 3.1 Commander（命令控制器）

**职责**：系统核心控制模块，负责：
- 电机解锁/上锁（Arming/Disarming）
- 飞行模式管理
- 故障安全（Failsafe）处理
- 校准流程控制
- 系统状态管理

**核心类**：`Commander` [commander.hpp](file:///workspace/src/modules/commander/Commander.hpp)

**主要组件**：
- `Failsafe` - 故障安全处理
- `FailureDetector` - 故障检测
- `HealthAndArmingChecks` - 健康状态和解锁检查
- `ModeManagement` - 飞行模式管理
- `UserModeIntention` - 用户模式意图处理

**关键方法**：
| 方法 | 功能 |
|------|------|
| `arm()` | 解锁电机 |
| `disarm()` | 上锁电机 |
| `handle_command()` | 处理车辆命令 |
| `run()` | 主循环 |
| `updateControlMode()` | 更新控制模式 |

---

### 3.2 EKF2（扩展卡尔曼滤波器）

**职责**：状态估计核心，融合多传感器数据：
- 姿态估计（四元数）
- 位置估计（本地/全局）
- 速度估计
- IMU 偏置估计
- 风速估计
- 地形高度估计

**核心类**：`Ekf` [ekf.h](file:///workspace/src/modules/ekf2/EKF/ekf.h)

**状态向量组成**：
```
State = [quaternion (4) | velocity (3) | position (3) | 
         gyro_bias (3) | accel_bias (3) | mag_I (3) | 
         mag_B (3) | wind_vel (2) | terrain (1)]
```

**支持的观测源**：
| 传感器 | 配置宏 | 融合类型 |
|--------|--------|----------|
| GPS | `CONFIG_EKF2_GNSS` | 位置、速度、航向 |
| 气压计 | `CONFIG_EKF2_BAROMETER` | 高度 |
| 磁力计 | `CONFIG_EKF2_MAGNETOMETER` | 航向 |
| 光流 | `CONFIG_EKF2_OPTICAL_FLOW` | 速度 |
| 激光测距 | `CONFIG_EKF2_RANGE_FINDER` | 离地高度 |
| 外部视觉 | `CONFIG_EKF2_EXTERNAL_VISION` | 位置、速度、航向 |
| 空速计 | `CONFIG_EKF2_AIRSPEED` | 空速、侧滑角 |

**核心方法**：
| 方法 | 功能 |
|------|------|
| `init()` | 初始化滤波器 |
| `update()` | 主更新循环 |
| `predictState()` | 状态预测 |
| `predictCovariance()` | 协方差预测 |
| `fuse()` | 通用融合接口 |
| `fuseHorizontalPosition()` | 水平位置融合 |
| `fuseVelocity()` | 速度融合 |
| `fuseMag()` | 磁力计融合 |

---

### 3.3 MAVLink 通信模块

**职责**：MAVLink 协议实现，负责：
- 与地面站通信
- 消息流管理
- 参数传输
- 任务管理
- 日志流传输

**核心文件**：[mavlink_main.h](file:///workspace/src/modules/mavlink/mavlink_main.h)

**关键组件**：
- `MavlinkReceiver` - 消息接收
- `MavlinkStream` - 消息流管理
- `MavlinkParameters` - 参数同步
- `MavlinkMission` - 任务管理
- `MavlinkFTP` - 文件传输

**支持的消息流**：
- ATTITUDE、GLOBAL_POSITION_INT、LOCAL_POSITION_NED
- HIGHRES_IMU、GPS_RAW_INT、BATTERY_STATUS
- 以及更多...

---

### 3.4 Sensors（传感器模块）

**职责**：传感器数据采集与预处理：
- IMU 数据融合
- 传感器校准管理
- 数据验证与投票
- 传感器选择

**核心文件**：[sensors.hpp](file:///workspace/src/modules/sensors/sensors.hpp)

**子模块**：
| 子模块 | 功能 |
|--------|------|
| `vehicle_imu` | IMU 数据处理 |
| `vehicle_gps_position` | GPS 数据处理 |
| `vehicle_magnetometer` | 磁力计数据处理 |
| `vehicle_air_data` | 空速数据处理 |
| `vehicle_optical_flow` | 光流数据处理 |
| `data_validator` | 数据验证器 |

---

### 3.5 多旋翼控制模块

#### 3.5.1 mc_rate_control（速率控制）

**职责**：姿态速率闭环控制

**核心类**：`MulticopterRateControl`

**控制结构**：
```
输入: 期望角速率
处理: PID控制器
输出: 姿态力矩设定值
```

#### 3.5.2 mc_att_control（姿态控制）

**职责**：姿态闭环控制

**核心类**：`MulticopterAttitudeControl`

**控制结构**：
```
输入: 期望姿态（四元数）
处理: P控制器计算角速率指令
输出: 角速率设定值 -> 传给 mc_rate_control
```

#### 3.5.3 mc_pos_control（位置控制）

**职责**：位置闭环控制，包含：
- 位置控制器
- 速度控制器
- 高度控制器
- 起飞控制器

**核心文件**：[MulticopterPositionControl.hpp](file:///workspace/src/modules/mc_pos_control/MulticopterPositionControl.hpp)

**子模块**：
| 子模块 | 功能 |
|--------|------|
| `PositionControl` | 位置/速度控制 |
| `GotoControl` | 定点导航控制 |
| `Takeoff` | 起飞控制 |

---

### 3.6 固定翼控制模块

#### 3.6.1 fw_rate_control（速率控制）

**职责**：固定翼姿态速率控制

#### 3.6.2 fw_att_control（姿态控制）

**职责**：固定翼姿态控制

**核心类**：`FixedwingAttitudeControl`

#### 3.6.3 fw_lateral_longitudinal_control（横向/纵向控制）

**职责**：固定翼航迹控制

**核心类**：`FwLateralLongitudinalControl`

**控制模式**：
- 高度保持
- 速度保持
- 航向保持
- 航线跟踪

---

### 3.7 Navigator（导航模块）

**职责**：高级导航任务管理：
- 任务执行
- 返航（RTL）
- 盘旋（Loiter）
- 降落（Land）
- 地理围栏（Geofence）

**核心文件**：[navigator.h](file:///workspace/src/modules/navigator/navigator.h)

**导航模式**：
| 模式 | 功能 |
|------|------|
| Mission | 任务执行 |
| RTL | 返航着陆 |
| Loiter | 定点盘旋 |
| Land | 自动降落 |
| Takeoff | 自动起飞 |
| GeoFence | 地理围栏 |

---

### 3.8 Control Allocator（控制分配器）

**职责**：将控制指令分配到执行器：
- 电机混控
- 执行器分组
- 优先级管理

**核心文件**：[ControlAllocator.hpp](file:///workspace/src/modules/control_allocator/ControlAllocator.hpp)

**分配算法**：
- 伪逆法（PseudoInverse）
- 顺序去饱和法（SequentialDesaturation）

---

## 4. 核心库说明

### 4.1 MathLib（数学库）

**职责**：基础数学工具

**文件位置**：[mathlib.h](file:///workspace/src/lib/mathlib/mathlib.h)

**组件**：
| 组件 | 功能 |
|------|------|
| `math/` | 基础数学函数 |
| `math/filter/` | 滤波器实现 |
| `matrix/` | 矩阵运算库 |

**滤波器类型**：
- `AlphaFilter` - 一阶低通滤波
- `LowPassFilter2p` - 二阶低通滤波
- `MedianFilter` - 中值滤波
- `NotchFilter` - 陷波滤波

---

### 4.2 Matrix Library（矩阵库）

**职责**：高性能矩阵运算

**核心类型**：
| 类型 | 说明 |
|------|------|
| `Matrix<T, M, N>` | 通用矩阵 |
| `SquareMatrix<T, N>` | 方阵 |
| `Vector<T, N>` | 向量 |
| `Vector2<T>` | 2维向量 |
| `Vector3<T>` | 3维向量 |
| `Vector4<T>` | 4维向量 |
| `Quaternion<T>` | 四元数 |
| `Dcm<T>` | 方向余弦矩阵 |
| `Euler<T>` | 欧拉角 |
| `AxisAngle<T>` | 轴角表示 |

**文件位置**：[src/lib/mathlib/matrix/](file:///workspace/src/lib/mathlib/matrix/)

---

### 4.3 ControlLib（控制库）

**职责**：控制器构建模块

**核心模块**：[blocks.hpp](file:///workspace/src/lib/controllib/blocks.hpp)

**可用模块**：
| 模块 | 功能 |
|------|------|
| `BlockP` | 比例控制器 |
| `BlockPI` | PI控制器 |
| `BlockPD` | PD控制器 |
| `BlockPID` | PID控制器 |
| `BlockIntegral` | 积分器 |
| `BlockDerivative` | 微分器 |
| `BlockLowPass` | 低通滤波器 |
| `BlockHighPass` | 高通滤波器 |
| `BlockLimit` | 限幅器 |
| `BlockDelay` | 延迟模块 |

---

### 4.4 Parameters（参数库）

**职责**：系统参数管理

**核心文件**：[param.h](file:///workspace/src/lib/parameters/param.h)

**参数层架构**：
```
┌─────────────────────┐
│   External Layer    │  外部参数源
├─────────────────────┤
│ DynamicSparseLayer  │  动态稀疏层
├─────────────────────┤
│ StaticSparseLayer   │  静态稀疏层
├─────────────────────┤
│  ExhaustiveLayer    │  穷举层
├─────────────────────┤
│   ConstLayer        │  常量层
└─────────────────────┘
```

**关键功能**：
- 参数存储与加载
- 参数类型支持（float、int、bool）
- 参数自动保存
- 参数远程访问

---

### 4.5 Events（事件系统）

**职责**：系统事件管理与日志

**核心文件**：[events.h](file:///workspace/src/lib/events/events.h)

**功能**：
- 事件发布/订阅
- 事件日志
- 事件过滤

---

### 4.6 uORB（微型对象请求代理）

**职责**：进程间通信中间件

**核心概念**：
- **Topic**：消息主题
- **Publication**：消息发布
- **Subscription**：消息订阅

**关键特性**：
- 异步消息传递
- 支持多订阅者
- 时间戳管理
- DDS 兼容

**常用 API**：
| API | 功能 |
|-----|------|
| `orb_advertise()` | 发布消息 |
| `orb_subscribe()` | 订阅消息 |
| `orb_copy()` | 复制消息 |
| `orb_check()` | 检查新消息 |

---

## 5. 消息定义（uORB Topics）

### 5.1 核心消息类型

#### 传感器消息
| 消息 | 说明 |
|------|------|
| `sensor_combined` | 组合传感器数据 |
| `sensor_gyro` | 陀螺仪数据 |
| `sensor_accel` | 加速度计数据 |
| `sensor_mag` | 磁力计数据 |
| `sensor_gps` | GPS 数据 |
| `sensor_baro` | 气压计数据 |
| `sensor_optical_flow` | 光流数据 |

#### 状态消息
| 消息 | 说明 |
|------|------|
| `vehicle_status` | 车辆状态 |
| `vehicle_attitude` | 姿态估计 |
| `vehicle_local_position` | 本地位置 |
| `vehicle_global_position` | 全局位置 |
| `vehicle_odometry` | 里程计 |
| `actuator_armed` | 执行器解锁状态 |

#### 控制消息
| 消息 | 说明 |
|------|------|
| `vehicle_attitude_setpoint` | 姿态设定值 |
| `vehicle_rates_setpoint` | 角速率设定值 |
| `vehicle_local_position_setpoint` | 本地位置设定值 |
| `actuator_controls` | 执行器控制指令 |
| `manual_control_setpoint` | 手动控制设定值 |

#### 系统消息
| 消息 | 说明 |
|------|------|
| `battery_status` | 电池状态 |
| `system_power` | 系统电源 |
| `cpuload` | CPU 负载 |
| `parameter_update` | 参数更新 |

---

## 6. 平台适配层

### 6.1 支持的平台

| 平台 | 说明 |
|------|------|
| **NuttX** | 嵌入式实时操作系统，用于硬件飞控 |
| **POSIX** | Linux/macOS，用于仿真和开发 |
| **QURT** | Qualcomm 实时操作系统 |
| **ROS 2** | ROS 2 接口支持 |

### 6.2 平台抽象

**文件位置**：[platforms/](file:///workspace/platforms/)

**抽象层**：
- `board_common.c` - 板级通用接口
- `i2c.cpp` / `spi.cpp` - 外设总线
- `px4_cli.cpp` - 命令行接口
- `px4_log.cpp` - 日志接口

---

## 7. 构建与运行

### 7.1 构建命令

```bash
# 克隆仓库
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# 构建 SITL（软件在环仿真）
make px4_sitl

# 构建特定硬件
make px4_fmu-v5_default

# 列出所有可用目标
make list_configs
```

### 7.2 运行仿真

```bash
# 启动 SITL 仿真
make px4_sitl gazebo-classic

# 或使用 Docker
docker run --rm -it -p 14550:14550/udp px4io/px4-sitl:latest
```

### 7.3 构建配置

**配置文件**：
- `Kconfig` - 内核配置
- `posix-configs/` - POSIX 配置
- `ROMFS/` - 只读文件系统内容

---

## 8. 测试框架

### 8.1 单元测试

**测试目录**：`test/` 和各模块内的 `test/` 目录

**测试框架**：Google Test (GTest)

**运行测试**：
```bash
# 构建测试配置
make px4_sitl_test

# 运行测试
cd build/px4_sitl_test
ctest
```

### 8.2 集成测试

**MAVSDK 测试**：[test/mavsdk_tests/](file:///workspace/test/mavsdk_tests/)

**测试类型**：
- `test_multicopter_basics` - 多旋翼基础测试
- `test_multicopter_mission` - 多旋翼任务测试
- `test_vtol_mission` - VTOL 任务测试

---

## 9. 关键技术特性

### 9.1 故障安全机制

**故障安全层次**：
1. **传感器故障检测** - 数据验证和投票
2. **通信故障检测** - 数据链丢失检测
3. **执行器故障检测** - 电机故障检测
4. **紧急降落** - 故障时自动降落

### 9.2 传感器融合

**融合策略**：
- 扩展卡尔曼滤波（EKF）
- 多传感器数据融合
- 自适应噪声协方差
- 故障检测与隔离（FDI）

### 9.3 控制架构

**分层控制**：
```
高层指令 → 位置控制 → 姿态控制 → 速率控制 → 执行器
```

---

## 10. 依赖关系

### 10.1 外部依赖

| 依赖 | 用途 |
|------|------|
| **MAVLink** | 通信协议 |
| **uORB** | 内部消息传递 |
| **Eigen** | 矩阵运算（内部实现） |
| **CMSIS-DSP** | 数字信号处理 |
| **TensorFlow Lite Micro** | 神经网络推理 |

### 10.2 模块依赖图

```
commander ─────────────────────────────┐
    │                                  │
    ├──> uORB (pub/sub)               │
    ├──> parameters                   │
    └──> events                       │
                                      │
ekf2 ─────────────────────────────────┤
    │                                  │
    ├──> sensors (sub)                │
    ├──> mathlib                      │
    └──> matrix                       │
                                      │
mc_att_control ───────────────────────┤
    │                                  │
    ├──> ekf2 (sub)                   │
    ├──> controllib                   │
    └──> control_allocator (pub)      │
                                      │
mavlink ──────────────────────────────┤
    │                                  │
    ├──> uORB (sub/pub)               │
    └──> commander (sub/pub)          │
                                      │
sensors ──────────────────────────────┘
    │
    ├──> drivers
    └──> data_validator
```

---

## 11. 开发流程

### 11.1 模块开发

**模块结构**：
```
src/modules/<module_name>/
├── CMakeLists.txt    # 构建配置
├── Kconfig           # 配置选项
├── module.yaml       # 模块元数据
├── <module>_main.cpp # 主入口
├── <module>.hpp      # 头文件
└── params.yaml       # 参数定义
```

**模块接口**：
```cpp
class MyModule : public ModuleBase, public ModuleParams
{
public:
    static int task_spawn(int argc, char *argv[]);
    static MyModule *instantiate(int argc, char *argv[]);
    void run() override;
    int print_status() override;
};
```

### 11.2 参数定义

**YAML 格式**：
```yaml
---
parameters:
  - name: MY_PARAM
    type: float
    default: 1.0
    min: 0.0
    max: 10.0
    description: My parameter description
```

---

## 12. 参考资料

| 资源 | 链接 |
|------|------|
| 官方文档 | https://docs.px4.io |
| 用户指南 | https://docs.px4.io/main/en/ |
| 开发者指南 | https://docs.px4.io/main/en/development/ |
| MAVLink 文档 | https://mavlink.io |
| uORB 文档 | https://docs.px4.io/main/en/middleware/uorb.html |

---

**文档版本**: PX4 Autopilot Code Wiki v1.0  
**生成时间**: 2026年6月  
**项目版本**: 基于最新 Git 版本