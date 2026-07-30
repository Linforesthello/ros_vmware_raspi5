# R1 机械臂 — CAN 协议设计

> 从对话历史提炼 · 2026-06-29

---

## 一、波特率统一

**全系统必须统一波特率**，否则无法通信。

| 节点 | 当前 | 目标 |
|:-----|:-----|:-----|
| STM32#1 MCLM (底盘) | 500k (Prescaler=4) | **500k** 不变 |
| STM32#2 SteeringArm (腕) | 500k (Prescaler=4) | **500k** 不变 |
| RS00 #1/#2 | 1M (待确认) | **改到500k** |
| PC CANable | slcan 可配 | **500k** (-s6) |

**RS00 改波特率方法:**
```bash
# Type 23: data[0]=波特率值 (6=500k, 8=1M)
cansend can0 1730FD01#0600000000000000  # 电机#1改到500k
cansend can0 1730FD02#0600000000000000  # 电机#2改到500k
# → 重新上电生效
```

> 建议**改 RS00 到 500k** 而非改 STM32 到 1M，因为 MCLM 已有完整代码且稳定运行，改波特率后要重新验证整个底盘控制逻辑。

---

## 二、CAN ID 分配总表

### 标准帧 11-bit

| ID | 方向 | 用途 | 节点 | 频率 | 继承自 |
|:--:|:----:|:-----|:----|:----:|:-------|
| **0x101** | 广播 | 全车急停 | 任意→ALL | 事件 | MCLM协议 |
| **0x125** | → | 底盘转向电机指令 | PC→MCLM | 偶发 | MCLM Group1 |
| **0x126** | → | 底盘动力电机指令 | PC→MCLM | 偶发 | MCLM Group1 |
| **0x225** | ↔ | 底盘转向状态查询 | PC↔MCLM | 按需 | MCLM协议 |
| **0x226** | ↔ | 底盘动力状态查询 | PC↔MCLM | 按需 | MCLM协议 |
| **0x325** | ← | 底盘转向状态上报 | MCLM→总线 | **50ms** | MCLM协议 |
| **0x326** | ← | 底盘动力状态上报 | MCLM→总线 | **50ms** | MCLM协议 |
| **0x130** | → | 机械臂关节指令 | PC→Arm | 偶发 | Steering协议 |
| **0x131** | → | 关节速度设置 | PC→Arm | 偶发 | Steering协议 |
| **0x230** | → | 机械臂状态查询 | PC→Arm | 按需 | Steering协议 |
| **0x330** | ← | 机械臂状态上报 | Arm→总线 | **50ms** | Steering协议 |
| **0x430** | → | 机械臂参数配置 | PC→Arm | 偶发 | Steering协议 |
| **0x4A0** | ← | **网关融合帧(车体速度)** | 网关→总线 | **10ms** | 新设计 |
| **0x4A1** | ← | **网关IMU姿态帧** | 网关→总线 | **10ms** | 新设计 |
| **0x4B0** | ← | 网关事件帧 | 网关→总线 | 事件 | 新设计 |
| **0x4FF** | → | 网关/N100间命令 | PC↔网关 | 偶发 | 新设计 |

### 扩展帧 29-bit（RS00 私有）

RS00 私有协议完全使用 29-bit 扩展帧空间，与上述标准帧无冲突。

```
RS00 扩展帧 ID 结构:
bit28~24: Type (1=运控, 2=反馈, 3=使能, 4=停止, 6=标零, 7=改ID, 17=读参, 18=写参, 23=改波特率, 24=主动上报)
bit23~8:  data_field (随Type含义不同)
bit7~0:   目标电机ID (1=肩, 2=肘)
```

---

## 三、帧格式详解

### MCLM 状态帧 (0x325/0x326, 现有不变)

```
Byte 0~1:  current_logic_speed  (int16 LE)
Byte 2~3:  accumulated_ticks    (uint16 LE, 里程计低16位)
Byte 4~5:  pwm_output           (int16 LE)
Byte 6:    target_logic_speed   (int8, -100~100)
Byte 7:    flags                (bit0=STALL, bit1=SATURATED)
```

### SteeringArm 指令帧 (0x130, 现有不变)

单关节模式 (LEGACY, data[0]=0x11):
```
Byte 0:    0x11 (SET)
Byte 1:    joint_id (1=J1, 2=J2, 3=Gripper, 预留0=J0)
Byte 2~3:  value (int16 LE, J1/J2=0.1°, Gripper=CCR脉冲)
Byte 4~7:  保留(填0)
```

多关节模式 (MULTI, data[0]=0x12, 推荐R1使用):
```
Byte 0:    0x12 (MULTI)
Byte 1:    bitmask (bit1=J1, bit2=J2, bit3=Gripper, bit0=J0预留)
Byte 2~3:  J1角度 (int16 LE, 0.1°)
Byte 4~5:  J2角度 (int16 LE, 0.1°)
Byte 6:    夹爪位置 (uint8, 0~200)
Byte 7:    J0速度 (int8, -100~100, 预留)
```

### SteeringArm 状态帧 (0x330, 待实现)

```
Byte 0~1:  J1 raw angle (uint16 LE, 0~16383, MT6701 14bit)
Byte 2~3:  J2 raw angle (uint16 LE, 0~16383)
Byte 4:    J1 当前角度 (int8, ±127°, 1°精度)
Byte 5:    J2 当前角度 (int8, ±127°)
Byte 6:    夹爪位置 (uint8, 0~200)
Byte 7:    标志位 (bit0=SERVO_ACTIVE, bit1=J1_IN_RANGE, bit2=J2_IN_RANGE)
```

### 网关融合帧 (0x4A0, 新设计, 10ms)

```
Byte 0:    帧序号 (uint8, 自增, N100检测丢帧)
Byte 1~2:  timestamp_tick (uint16, 网关tick, 10ms分辨率)
Byte 3~4:  车体 vx (int16, 0.01m/s, 范围 ±10m/s)
Byte 5~6:  车体 vy (int16, 0.01m/s, 范围 ±10m/s)
Byte 7:    车体 omega (int16, 0.01rad/s, 范围 ±30rad/s) — 高8位
```

### 网关IMU帧 (0x4A1, 新设计, 10ms)

```
Byte 0~1:  四元数 w (int16, 0.001精度)
Byte 2~3:  四元数 x (int16, 0.001精度)
Byte 4~5:  四元数 y (int16, 0.001精度)
Byte 6~7:  四元数 z (int16, 0.001精度)
```

### 网关事件帧 (0x4B0, 事件触发)

```
Byte 0:    事件类型 (0x01=堵转, 0x02=通信丢失, 0x03=急停)
Byte 1:    节点ID (0=网关, 1=MCLM转向, 2=MCLM动力, 3=SteeringArm)
Byte 2~3:  参数 (如电机当前值)
Byte 4~7:  保留
```

---

## 四、总线负载估算

| 帧来源 | ID | 频率 | 帧率 | 带宽占用 |
|:-------|:---|:----:|:----:|:--------:|
| MCLM转向状态 | 0x325 | 50ms | 20fps | ~0.5% |
| MCLM动力状态 | 0x326 | 50ms | 20fps | ~0.5% |
| Arm状态 | 0x330 | 50ms | 20fps | ~0.5% |
| 网关融合帧1 | 0x4A0 | **10ms** | **100fps** | ~2.5% |
| 网关融合帧2 | 0x4A1 | **10ms** | **100fps** | ~2.5% |
| 控制指令(合计) | 变化 | 偶发 | ~20fps | ~0.5% |
| RS00(扩展帧) | 变化 | 按需 | ~20fps | ~0.5% |
| **合计** | | | **~300fps** | **~7.5%** |

500kbps 下满载约 3300fps，当前 300fps = **9% 负载，余量充足**。

---

## 五、MCLM 固件 CAN 配置

| 参数 | 当前值 | 说明 |
|:-----|:-------|:-----|
| CAN实例 | CAN1 | — |
| 引脚 | **PB8(RX), PB9(TX)** | **重映射 CAN1** (`__HAL_AFIO_REMAP_CAN1_2()`) |
| 波特率 | **500k** (Prescaler=4, BS1=13, BS2=4) | 36MHz APB1 → 500k |
| 过滤器 | Bank 0, 掩码全0(硬件放行) | 软件过滤在 `can_filter.c` |
| 中断 | USB_LP_CAN1_RX0_IRQn, 优先级5 | — |
| 自动重传 | ENABLE | 硬件重传直到收到ACK |
| ID组 | **Group 1**: 0x125/0x126 系列 | 当前使用 |

### CAN_ID 可切换组

```
Group 1 (当前默认):  0x125/0x225/0x325  (转向)
                     0x126/0x226/0x326  (动力)

Group 2 (备选):      0x123/0x223/0x323  (转向)
                     0x124/0x224/0x324  (动力)
```

切换方法: 改 `app_config.h` 中 `CAN_ID_GROUP` 值。

---

## 六、SteeringArm CAN 配置

| 参数 | 值 |
|:-----|:-----|
| CAN实例 | CAN1 |
| 引脚 | **PA11(RX), PA12(TX)** (默认, 非重映射) |
| 波特率 | 500k (Prescaler=4, BS1=13, BS2=4) |
| 协议ID | 0x130/0x131/0x230/0x330/0x430 |
| 兼容性 | 与 MCLM 同波特率，同总线，不同 ID 空间，互不冲突 |
