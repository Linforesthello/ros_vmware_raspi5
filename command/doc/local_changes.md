# PC 直连 CAN 控制舵轮底盘 — 开发记录

> 日期: 2026-07-03~04
> 目的: PC 通过 USB-CAN 适配器直接控制 3_MCLM_t2 电机控制器，实现舵向角度闭环 + 全车运动学

---

## 文件清单

| 文件 | 说明 |
|------|------|
| `test_can_comms.py` | CAN 通信验证 — 监听状态帧解码 |
| `dashboard.py` | 实时仪表盘 — 显示 4 单元 × 2 电机状态 |
| `measure_steering_ticks.py` | 测转向电机 ticks/圈（单单元） |
| `measure_ticks_per_rev.py` | 测编码器 ticks/圈辅助工具 |
| `auto_measure_ticks.py` | 自动发命令测 ticks 变化 |
| `steering_control.py` | 舵向角度 PID 控制（单单元） |
| `chassis_control.py` | 全车底盘运动学控制 |

---

## Step 1: CAN 通信验证

**文件**: `test_can_comms.py`

验证了 MCLM_t2 电机控制器以 20Hz 主动上报状态帧 (0x321~0x328)：
- 全部 8 个 CAN ID 都能收到
- 状态帧解码：speed/ticks/pwm/target/flags

**CAN 接口**: `can0` (socketcan, slcand 驱动)

---

## Step 2: 编码器标定

### 转向电机 ticks/圈

实测数据（发 speed=20 低速，转一圈看 ticks 变化）：

| 单元 | 测试1 | 测试2 | 均值 | 每度 |
|------|-------|-------|------|------|
| UNIT1 (0x121) | 7773 | 8071 | **7922** | 22.0 |
| UNIT2 (0x123) | 9375 | 9307 | **9341** | 25.9 |
| UNIT3 (0x125) | 9420 | 9106 | **9263** | 25.7 |
| UNIT4 (0x127) | 9328 | 9423 | **9375** | 26.0 |

UNIT2~4 一致性较好 (~9300)，UNIT1 偏低 (~7922) — 机械公差。

### 驱动电机 ticks/圈

动力轮手动转一圈测得: **63156 ticks/圈**

---

## Step 3: 舵向角度 PID 控制

**文件**: `steering_control.py`

- P 控制器: Kp=0.8, max_speed=60, dead_zone=2°
- 速度模式/角度模式可切换
- `cal` 命令：当前位置归零
- 支持 `--unit 1~4` 选择控制哪个单元

### 控制效果
- UNIT1/3/4: 角度控制精准
- UNIT2: 需要 `cal` 归零后正常

---

## Step 4: 全车运动学

**文件**: `chassis_control.py`

输入 `vx vy omega` → 运动学逆解 → 4 单元转向角度 + 驱动速度

**车轮坐标**（y正方向=左侧）:
```
UNIT1 前左  (0.30,  0.25)
UNIT2 前右  (0.30, -0.25)
UNIT3 后左  (-0.30,  0.25)
UNIT4 后右  (-0.30, -0.25)
```

**命令格式**:
```
0.5 0 0        → 前进 0.5 m/s
0 0.3 0        → 横移 0.3 m/s
0 0 0.3        → 原地旋转 (omega 符号待确认)
stop           → 全车停止
cal            → 全部单元归零
```

### 已知问题

原地旋转 (`0 0 0.3`) 时 UNIT1 +90° 偏差、UNIT2 -90° 偏差：
- 原因：y 坐标符号或 omega 符号与车体坐标系不匹配
- 待修复：尝试交换公式为 `vx + omega*y, vy - omega*x`

---

## Step 5: uint16 回绕修复

### 问题

CAN 状态帧的 `accumulated_ticks` 只传低 16 位 (uint16)。当电机连续运动超过 65535 ticks 后，CAN 帧值回绕 65535→0，导致 `ticks_to_deg` 计算的**角度跳变**。

固件 int32:  `0 → 1000 → ... → 65535 → 65536`
CAN uint16:  `0 → 1000 → ... → 65535 →     0` ← 角度从 360° 跳回 0°

### 修复

添加 uint16 解包机制：
```python
def unwrap_ticks(raw):
    """检测 65535→0 或 0→65535 跳变，维护连续 abs_ticks"""
    diff = raw - prev_raw
    if diff > 32768:    diff -= 65536   # 正向回绕
    if diff < -32768:   diff += 65536   # 反向回绕
    abs_ticks += diff
```

角度计算改用 `abs_ticks`（连续 int32）→ 不再跳变。

---

## 参考文档

- [deepseek_can.md](../../../3_MotorControl_LowMachine/lin_cmake/3_MCLM_t2/doc/deepseek_can.md) — CAN 协议详述
- [Func1_planMt6701.md](../../../3_MotorControl_LowMachine/lin_cmake/3_MCLM_t2/doc/Function/Func1_planMt6701.md) — 角度闭环计划
- [chassis_model.md](../../doc/chassis_model.md) — 底盘构型设计
