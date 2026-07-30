# ROS2 集成与部署规划

> 2026-07-04
> 目标: 将底盘 + 机械臂接入 ROS2，部署到车载计算机
> 
> 系统包含:
> - **四舵轮底盘** (4×MCLM_t2, 11-bit CAN 标准帧 0x121~0x328)
> - **RS00 机械臂** (2×RS00 准直驱电机, 29-bit CAN 扩展帧)
> - **舵机腕部+夹爪** (3_SteeringArm_t1 STM32, 11-bit CAN 标准帧 0x130~0x430)
> - **全部在同一条 CAN 总线** (1Mbps ✅ 已全线统一)

---

## 一、硬件平台选择

### 对比

| 方面 | Mini PC (x86) | Raspberry Pi 4B | Raspberry Pi 5 |
|------|--------------|----------------|----------------|
| CPU | N100/N305 (4核~3.4GHz) | Cortex-A72 4核 1.8GHz | Cortex-A76 4核 2.4GHz |
| 内存 | 8~16GB DDR4/DDR5 | 2~8GB LPDDR4 | 4~8GB LPDDR4X |
| USB | USB3.0 × 4 | USB3.0 × 2 | USB3.0 × 2 |
| CAN 适配 | USB-CANable 1个够用 | 同左 | 同左 |
| ROS2 Humble | ✅ 流畅 | ⚠ 勉强 | ✅ 流畅 |
| ROS2 Jazzy | ✅ 原生 | ❌ 不支持 | ⚠ 需编译 |
| 功耗 | 15~25W | 5~8W | 8~15W |
| 价格 | ~¥600 (准系统) | ~¥350 (4GB) | ~¥500 (8GB) |

### 结论

| 场景 | 推荐 | 理由 |
|------|------|------|
| **开发调试** | **Mini PC (N100)** | x86 生态好, 编译快, USB 口多, 不担心功耗 |
| **车载部署** | **RPi 5 (8GB)** | 性能够 ROS2, 功耗低, 体积小 |
| 低成本部署 | RPi 4B (4GB) | 勉强能跑, 但编译和响应慢 |

**建议先用 Mini PC 开发，验证通过后移到 RPi 5 部署。**

---

## 二、完整系统架构

### 2.1 物理连接

```
                    ┌──────────────────┐
                    │  车载计算机       │
                    │  (Mini PC / RPi) │
                    │                  │
                    │   USB ── CANable │
                    └────────┬─────────┘
                             │ slcand -s8 (1Mbps)
                             ▼
                     ┌──────────────┐
                     │  CAN Bus     │
                     │  1Mbps ✅    │
                     └──────┬───────┘
                            │
       ┌────────────────────┼────────────────────┐
       │                    │                    │
       ▼                    ▼                    ▼
┌──────────────┐   ┌──────────────┐   ┌──────────────────┐
│ MCLM_t2 ×4   │   │ RS00 #1 (肩) │   │ STM32#2          │
│ 底盘电机     │   │ RS00 #2 (肘) │   │ 3_SteeringArm_t1 │
│ CAN: 0x121~  │   │ CAN: 29-bit  │   │ 舵机腕部+夹爪    │
│      0x328   │   │ 扩展帧       │   │ CAN: 0x130~0x430 │
│ 1Mbps        │   │ 1Mbps        │   │ 1Mbps            │
└──────────────┘   └──────────────┘   └──────────────────┘
```

### 2.2 CAN 总线协议共存

三种协议在同一条总线共存（1Mbps），互不冲突：

| 协议 | 帧格式 | 用途 | ID 空间 | 说明 |
|------|--------|------|---------|------|
| **底盘 MCLM** | 11-bit 标准帧 | 4个舵轮转向+驱动 | 0x121~0x128 (CMD) | 每单元一对 ID |
| | | 状态反馈 | 0x321~0x328 (STATUS) | 20Hz 主动上报 |
| | | 广播命令 | 0x101~0x103 | 急停/全车 |
| **手腕 STM32#2** | 11-bit 标准帧 | 舵机角度控制 | **0x130** (ARM_CMD) | 单关节+多关节模式 |
| | | 参数配置 | **0x430** (ARM_CONFIG) | 回中/设速/锁定 |
| | | 状态上报 | **0x330** (ARM_STATUS) | 50ms |
| **RS00 电机** | **29-bit 扩展帧** | 肩/肘关节控制 | 扩展帧独立空间 | 私有协议 |
| | | 使能/停止 | Type 3/4 | |
| | | 运动控制 | Type 1 (pos+vel+Kp+Kd) | |
| **网关融合** | 11-bit 标准帧 | 车体速度 | **0x4A0** | 100Hz 融合帧 |
| | | IMU 四元数 | **0x4A1** | 100Hz 融合帧 |

> 29-bit 扩展帧和 11-bit 标准帧在同一条 CAN 总线上完全兼容，
> 硬件仲裁阶段即区分，不会产生 ID 冲突。

### 2.3 RS00 29-bit 扩展帧协议

RS00 电机使用私有 29-bit 扩展帧协议，ID 结构：

```
bit[28:24] ─ 通信类型 (5bit): 1=运动控制, 2=应答, 3=使能, 4=停止...
bit[23:8]  ─ 数据字段 (16bit): 含义因类型而异
bit[7:0]   ─ 目标电机 CAN ID (8bit): 1=肩, 2=肘, 0xFD=PC
```

**Type 1 运动控制（核心）** — 8 字节数据区：
```
[0-1] 目标位置 (uint16)   → 映射 ±720°
[2-3] 目标速度 (uint16)   → 映射 ±1891 °/s
[4-5] Kp (uint16)         → 映射 0~500
[6-7] Kd (uint16)         → 映射 0~5
```

Python 控制类：`rs00_arm.py` 的 `RS00Arm` 类封装双电机操作。

### 2.4 双通道 CAN 架构

同一条物理总线上定义两个逻辑通道（参考 `Lin_rs00/doc/dual_channel_can.md`）：

| 通道 | CAN ID 范围 | 频率 | 内容 | 用途 |
|------|------------|:----:|------|------|
| **低速通道** | 0x1xx/0x2xx/0x3xx | 20~50Hz | 各子系统原始状态 | 交叉验证、日志 |
| **高速通道** | **0x4A0/0x4A1** | **100Hz** | 网关融合数据 | 实时控制主回路 |

```
PC (ROS2, ~50ms 策略层)
  │ CAN 指令
  ▼
5_ChassisController_t1 (网关/融合, 10ms 实时层)
  │ CAN 0x4A0 (vx,vy,omega 100Hz)
  │ CAN 0x4A1 (IMU 四元数 100Hz)
  ▼
实时控制回路
```

### 2.5 节点架构

```
┌──────────────────────────────────────────────────────────┐
│                   ROS2 网络                              │
│                                                          │
│  /cmd_vel          /arm/joint_cmd     /arm/joint_state   │
│  (Twist)           (Float64MultiArray) (JointState)      │
│      │                   │                   ▲           │
│      ▼                   ▼                   │           │
│  ┌──────────┐    ┌──────────────┐           │           │
│  │ chassis  │    │  arm_node    │───────────┘           │
│  │ _node    │    │              │                        │
│  │          │    │ rs00_arm.py  │←→ RS00 29bit 扩展帧    │
│  │ chassis  │    │ rs00_control │                        │
│  │ _control │    └──────┬───────┘                        │
│  │ (运动学)  │           │                                │
│  └─────┬────┘           ├──→ 0x130/0x430 (腕部STM32#2)   │
│        │                │                                │
│  mclm_can.py ──→ 0x121~0x328 (底盘 MCLM)                 │
│        │                                                │
│        同一条 CAN 总线 (1Mbps)                            │
└──────────────────────────────────────────────────────────┘
```

---

## 三、各节点详细设计

### 3.1 底盘节点 `chassis_node`

```
输入:  /cmd_vel (geometry_msgs/Twist)
      vx = msg.linear.x     # m/s, 正=前进
      vy = msg.linear.y     # m/s, 正=右移
      omega = msg.angular.z # rad/s, 正=左旋

核心逻辑: (从 chassis_control.py 复用)
  1. inv_kinematics(vx, vy, omega) → 4×角度+速度
  2. 4路 PID 控制转向电机到位
  3. 4路驱动电机速度控制

发布:  /chassis/status (String)  — 各单元角度/速度/状态
       /chassis/odom (nav_msgs/Odometry) — 里程计 [Phase 2]
服务:  /chassis/cal   → 归零
       /chassis/steer → 仅转向不移动
```

**代码复用关系：**
```
chassis_control.py (现有 CLI)    →   chassis_node.py (ROS2)
  ├─ inv_kinematics()                   复用
  ├─ PID 控制循环                       复用
  ├─ CLI input()                        替换为 /cmd_vel subscriber
  ├─ print_status()                     替换为 /chassis/status publisher
  ├─ steer/cal/stop 命令                替换为 service
  └─ drive speed 计算                   复用
```

### 3.2 RS00 机械臂节点 `arm_node`

```
输入:  /arm/joint_target (std_msgs/Float64MultiArray)
      data[0] = 肩部角度 (°)
      data[1] = 肘部角度 (°)

核心逻辑: (从 rs00_arm.py 复用)
  1. 角度限位检查 (肩: -40°~+220°, 肘: -70°~+110°)
  2. 构建 RS00 运动控制帧 (Type 1, 29-bit 扩展帧)
  3. CAN 发送 + 接收应答
  4. 角度/速度/Kp/Kd 编码

发布:  /arm/joint_state (sensor_msgs/JointState)
      position[0] = 肩部当前角度
      position[1] = 肘部当前角度

服务:  /arm/enable   → 使能电机
       /arm/disable  → 停止电机
       /arm/set_zero → 设机械零位
       /arm/set_mode → 切换运动控制/CSP模式
```

**代码复用关系：**
```
rs00_arm.py (现有 CLI)           →   arm_node.py (ROS2)
  ├─ RS00Arm 类 (set_angles等)          复用
  ├─ 角度限位                            复用
  ├─ CLI / 交互模式                      替换为 ROS2 subscriber
  └─ monitor.py 状态显示                 替换为 JointState publisher
```

### 3.3 舵机腕部节点 `wrist_node`（可并入 arm_node）

```
输入:  /wrist/target (std_msgs/Float64MultiArray)
      data[0] = J1 角度 (°)   [-150°~+150°]
      data[1] = J2 角度 (°)   [-150°~+150°]
      data[2] = 夹爪 (0~100%)

核心逻辑:
  1. 构建 CAN 0x130 帧 (11-bit 标准帧)
  2. 发送给 3_SteeringArm_t1 STM32
  3. 接收 0x330 状态帧

发布:  /wrist/status (String)
```

### 3.4 串口桥接节点 `serial_bridge`（可选）

遥控器 / 上位机通过 UART 发命令时：

```
输入:  UART (115200 8N1) → 按协议解析
输出:  /cmd_vel            → 发给底盘节点
       /arm/joint_target   → 发给机械臂节点

协议格式 (复用 chassis_control.py 的命令风格):
  "0.5 0 0"         → /cmd_vel vx=0.5
  "0 0 0.3"         → /cmd_vel omega=0.3
  "arm 90 -45"      → /arm/joint_target [90, -45]
  "arm en"          → /arm/enable
```

---

## 四、实施阶段

### Phase 1: 基础设施 (1天)

- [ ] 车载计算机 OS 安装 (Ubuntu 24.04 / Raspberry Pi OS)
- [ ] ROS2 Humble 安装
- [ ] python-can 安装, slcand 配置
- [ ] 验证 CAN 总线全部节点在线 (1Mbps ✅ 已统一)

### Phase 2: 底盘 ROS2 节点 (1天)

- [ ] 创建 `chassis_node.py`
- [ ] 移植 `chassis_control.py` 逻辑
- [ ] `/cmd_vel` → CAN 全链路测试
- [ ] 验证前进/后退/横移/旋转

### Phase 3: 机械臂 ROS2 节点 (1天)

- [ ] 创建 `arm_node.py`
- [ ] 移植 `rs00_arm.py` 控制逻辑
- [ ] RS00 肩/肘 CAN 控制
- [ ] 舵机腕部 0x130 CAN 控制

### Phase 4: 串口桥接 + 遥控 (0.5天)

- [ ] 创建 `serial_bridge.py`
- [ ] 定义串口协议
- [ ] 遥控器 → 串口 → ROS2 → CAN 全链路

### Phase 5: 状态反馈与里程计 (2天)

- [ ] `/chassis/status` 定期发布
- [ ] `/arm/joint_state` 定期发布
- [ ] 编码器 ticks → 里程计 → `/odom`
- [ ] robot_localization 融合

### Phase 6: 导航集成 (1~2周)

- [ ] Nav2 navigation stack 配置
- [ ] MoveIt 机械臂规划集成
- [ ] 整机自主导航+操作

---

## 五、CAN 总线负载估算

### 5.1 现有负载

| 消息 | 数量 | 每帧字节 | 频率 | 帧/s | 带宽 |
|------|:----:|:--------:|:----:|:----:|:----:|
| 底盘状态 (0x321~8) | 8 | 8 | 20Hz | 160 | 12.8% |
| 底盘命令 (0x121~8) | 8 | 8 | 20Hz | ≤160 | ≤12.8% |
| RS00 控制 (29bit) | 2 | 8 | 50Hz | 100 | 8% |
| RS00 应答 (29bit) | 2 | 8 | 50Hz | 100 | 8% |
| 腕部状态 (0x330) | 1 | 8 | 20Hz | 20 | 1.6% |
| 腕部命令 (0x130) | 1 | 8 | 20Hz | ≤20 | ≤1.6% |

**峰值总计: ~560 帧/s, 带宽占用 ~45%** ← 安全

### 5.2 CAN 总线状态

| 设备 | 波特率 | 状态 |
|------|:------:|:----:|
| CAN 总线 (CANable slcand -s8) | **1Mbps** | ✅ 已统一 |
| MCLM_t2 ×4 底盘 | **1Mbps** | ✅ |
| RS00 电机 ×2 肩/肘 | **1Mbps** | ✅ |
| SteeringArm STM32#2 腕部 | **1Mbps** | ✅ |

---

## 六、部署建议

### Mini PC (开发阶段)

- 直接插 USB-CANable，用现有 Python 脚本开发
- ROS2 Humble 本地安装
- 底盘和机械臂同时控制

### RPi 5 (车载部署)

- 供电: 5V 5A USB-C (或车载 12V→5V DC-DC)
- CAN 适配: USB-CANable 或 MCP2515 SPI CAN 模块
- slcand 开机自启 (systemd service)
- chassis_node 和 arm_node 开机自启

### 文件结构部署

```
~/ros2_ws/src/
├── chassis_bringup/
│   ├── chassis_node.py      # 底盘 ROS2 节点
│   ├── chassis_control.py   # 运动学+PID 核心 (复用)
│   └── mclm_can.py          # CAN 协议封装 (复用)
│
├── arm_bringup/
│   ├── arm_node.py          # 机械臂 ROS2 节点  
│   ├── rs00_arm.py          # RS00 控制类 (复用)
│   └── rs00_control.py      # RS00 CAN 协议 (复用)
│
├── serial_bridge/
│   └── serial_bridge.py     # 串口→ROS2 桥接
│
└── can_common/
    └── mclm_can.py          # 共享 CAN 库
```

---

## 七、检查清单

### 硬件

- [ ] 选择并采购车载计算机 (Mini PC / RPi 5)
- [ ] USB-CANable（已有）
- [ ] 车载 12V→5V/12V DC-DC 电源
- [ ] CAN 总线终端电阻 120Ω ×2（已有）

### 通信

- [ ] CAN 总线 1Mbps ✅ 已全线统一
- [ ] CANable slcand -s8 1Mbps ✅

### 软件

- [ ] OS 安装 + ROS2
- [ ] python-can + slcand 配置
- [ ] chassis_node.py 编写 + 测试
- [ ] arm_node.py 编写 + 测试
- [ ] serial_bridge.py (可选)
- [ ] 开机自启 (systemd)

### 联调

- [ ] 底盘单独控制
- [ ] 机械臂单独控制
- [ ] 底盘+机械臂同时控制
- [ ] 串口遥控全链路
- [ ] 导航集成
