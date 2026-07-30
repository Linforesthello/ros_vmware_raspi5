# RS00 双电机机械臂 — 操作手册

> 版本: v2.0 | 日期: 2026-07-10
> 适用: 肩肘串联双关节机械臂（单/双电机）

---

## 1. 上电与启动

### 1.1 硬件接线顺序

**先接 CAN，后上电，断电顺序相反。**

```
第一步: 电机 CAN_H/L -- CANable -- USB -- 电脑
第二步: 48V 电源 -- 电机 XT30 接口
```

> CANable 的 120Ω 终端电阻必须开启。
> 多个电机并联在同一对 CAN_H/L 线上。

### 1.2 启动 CAN 接口

```bash
# 交互式
CanCmd
# 选 1) 1Mbps -> can0 UP

# 手动
sudo slcand -o -c -s8 /dev/ttyACM0 can0
sudo ip link set can0 up
```

验证：
```bash
ip link show can0            # state UP
cansend can0 0300FD01#       # 使能电机 #1 → 咔一声
cansend can0 0400FD01#       # 停止
```

### 1.3 启动监控（可选）

```bash
cd ~/Lin_workspace/Lin_rs00
python3 monitor.py
```

> 纯被动监听，**绝不发送任何指令**，不干扰外部控制。
> 可与其他终端同时使用，互不冲突。

---

## 2. 标准操作流程

### 2.1 完整流程

```
上电 → 启动 CAN → 使能 → 标零 → CSP 位置模式 → 走固定点 → 归零 → 停止
```

> **推荐全 CSP 工作流**：放弃运控模式 (Type 1)，全部走 CSP 位置模式。
> 精度 0.1°、不调参、掰不动、自动扛自重。详见第 7 节。

### 2.2 双电机模式（肩#1 + 肘#2）— CSP 全流程

```python
from rs00_arm import RS00Arm

arm = RS00Arm()                     # 默认肩#1 肘#2
arm.enable()                        # 使能两个电机

# 标零（当前位置 → 用户角度 0°）
arm.set_zero()

# 切 CSP 位置模式（电机内部位置环，精度高、掰不动）
arm.enable_csp(speed_limit=2)

# 走固定点
arm.set_angles_csp(45, 20).verify()
arm.set_angles_csp(0, 0).verify()
arm.set_angles_csp(-45, -30).verify()

# 归零
arm.set_angles_csp(0, 0).verify()
arm.disable()
```

### 2.3 单电机模式（只接 1 个）

```python
from rs00_arm import RS00Arm

# elbow_id=None 跳过肘部，所有肘部操作自动忽略
arm = RS00Arm(elbow_id=None)
arm.enable()                        # 只使能肩膀#1
arm.set_zero()

arm.set_angles(90, 0, kp=150, kd=1).verify()

arm.enable_csp(speed_limit=3)
arm.set_angles_csp(90, 0).verify()

arm.home().verify()
arm.disable()
```

### 2.4 链式调用

```python
RS00Arm().enable().set_zero().set_angles(90, -45, kp=150).verify().sleep(2).home().disable()
RS00Arm(elbow_id=None).enable().set_zero().set_angles(90, 0, kp=150).verify().disable()
```

### 2.5 上下文管理器

```python
# 双电机
with RS00Arm() as arm:
    arm.set_zero()
    arm.set_angles(90, -45, kp=150).verify()
    arm.enable_csp()
    arm.set_angles_csp(90, -45)

# 单电机
with RS00Arm(elbow_id=None) as arm:
    arm.set_zero()
    arm.set_angles(45, 0, kp=150).verify()
# 退出 with 自动 disable
```

---

## 3. 两种控制模式对比

| 特性 | 运控模式 (Type 1) | CSP 模式 |
|:-----|:-----------------|:---------|
| **原理** | PD 力矩控制，PC 发 Kp/Kd | 电机内部位置环闭环 |
| **调用** | `set_angles()` | `enable_csp()` + `set_angles_csp()` |
| **刚度** | 取决于 Kp 值 | 高（内部 Kp=40） |
| **扛自重** | 需 Kp≥150（肩膀） | 自动 |
| **到位精度** | 有稳态误差 | 精度高 |
| **速度** | 响应快 | 可调（speed_limit） |
| **限位保护** | ✅ 有 | ✅ 有（v2.0 新增） |
| **标零偏移** | ✅ 自动 | ✅ 自动（v2.0 修复） |

### 运控模式（适合快速移动）

```python
arm.set_angles(90, -45, kp=150, kd=1)
```

| 关节 | 推荐 Kp | 说明 |
|:-----|:-------|:------|
| 肩膀 | **150** | 扛手臂自重，Kp 太小拉不动 |
| 肘部 | **50** | 负载小，Kp 可低 |

### CSP 模式（适合到位后锁死）

```python
arm.enable_csp(speed_limit=3)         # 3 rad/s ≈ 172°/s
arm.set_angles_csp(90, -45).verify()

# 切回运控
arm.enable_operation()
```

---

## 4. 分场景操作

### 4.1 场景：标定机械零点

```python
# 1. 手动把臂摆到机械零点位置（肩肘对齐）
# 2. 执行标零
arm.set_zero()
# 输出: [ZERO] 肩膀零点=xxx.x°  肘部零点=xxx.x°
# 3. 之后所有角度以此为零点基准
```

### 4.2 场景：定位到目标角度

```python
# 运控模式
arm.set_angles(90, -45, kp=150, kd=1).verify(settle_time=3)
# 输出:
#   [VERIFY] shoulder: 目标=90 实际=89.8 误差=0.2 ✅
#   [VERIFY] elbow: 目标=-45 实际=-44.4 误差=0.6 ✅
```

### 4.3 场景：到位后锁死

```python
arm.enable_csp(speed_limit=3)
arm.set_angles_csp(90, -45).verify()
# CSP 下电机内部锁定，掰不动
```

### 4.4 场景：单关节独立控制

```python
arm.set_shoulder(45, kp=150).verify()
arm.set_elbow(-30, kp=50).verify()
```

### 4.5 场景：限位调整

```python
# 当前默认限位：
#   肩膀: -85° ~ +85°  （电机原始 170°~340°，中位 255° 标零后）
#   肘部: -70° ~ +110°（待实测校准）

# 运行时临时修改
arm.set_limits(shoulder_min=-90, shoulder_max=90)
arm.set_limits(elbow_min=-80, elbow_max=80)

# 永久修改：编辑 rs00_arm.py 第 35-38 行 DEFAULT_SHOULDER_MIN/MAX
```

### 4.6 场景：紧急停止

```python
arm.stop()       # 立即 disable 两个电机
arm.disable()    # 同上
```

---

## 5. 状态监控

### 5.1 Python 读状态

```python
state = arm.get_state()
state['shoulder']['position']      # 肩膀当前角度 (°)
state['shoulder']['velocity']      # 当前速度 (°/s)
state['shoulder']['torque']        # 当前力矩 (N.m)
state['shoulder']['temperature']   # 温度 (°C)
state['elbow']                     # 肘部相同结构，单电机时=None
```

### 5.2 实时监控终端

```bash
cd ~/Lin_workspace/Lin_rs00

# 两个电机
python3 monitor.py

# 只监控特定的
python3 monitor.py --id 1      # 只肩膀
python3 monitor.py --id 2      # 只肘部
```

显示格式：
```
肩膀#1:  89.8°  +0.2°/s  0.001Nm  34.0°C  实时  100ms
肘部#2: -44.7°  -0.1°/s -0.104Nm  33.0°C  实时  50ms
```

字段说明：

| 字段 | 含义 |
|:-----|:------|
| 89.8° | 当前角度 |
| +0.2°/s | 当前速度 |
| 0.001Nm | 当前力矩 |
| 34.0°C | 温度 |
| 实时 | 最近收到的是电机主动上报/应答帧 |
| 100ms | 距上次更新的毫秒数 |

### 5.3 指令到位验证

```python
result = arm.set_angles(90, -45, kp=150).verify(settle_time=2, tolerance=5)
result['success']       # True/False
result['shoulder']     # {'target':90, 'actual':89.8, 'error':0.2, 'success':True}
```

---

## 6. 快速参考

### 6.1 实例化

```python
arm = RS00Arm()                              # 双电机: 肩#1 肘#2
arm = RS00Arm(elbow_id=None)                 # 单电机: 只肩#1
arm = RS00Arm(shoulder_id=1, elbow_id=2)     # 显式指定 ID
```

### 6.2 控制指令

```python
arm.enable()                                 # 使能
arm.disable()                                # 停止
arm.set_zero()                               # 当前位置 → 0°

# 运控模式（Type 1）
arm.set_angles(肩°, 肘°, kp=150, kd=1)      # 双关节
arm.set_shoulder(°, kp=150)                  # 单关节：肩膀
arm.set_elbow(°, kp=50)                      # 单关节：肘部
arm.home(vel=30)                             # 归零 (0°, 0°)

# CSP 模式
arm.enable_csp(speed_limit=3)                # 切 CSP
arm.set_angles_csp(肩°, 肘°)                # CSP 目标角度
arm.enable_operation()                       # 切回运控

# 验证
result = arm.verify(settle_time=2, tolerance=5)
state = arm.get_state()

# 限位
arm.set_limits(shoulder_min=-85, shoulder_max=85)
```

### 6.3 完整指令集

所有指令在 `rs00_control.py` 中实现，`RS00Arm` 类封装上层调用。

#### 基础控制

| 操作 | 类型 | Python API | cansend 示例 |
|:-----|:----:|:-----------|:-------------|
| **使能** | Type 3 | `motor_enable(iface, mid)` | `cansend can0 0300FD01#` |
| **停止** | Type 4 | `motor_disable(iface, mid)` | `cansend can0 0400FD01#` |
| **运控指令** | Type 1 | `motor_control(iface, mid, pos, vel, kp, kd, torque)` | `cansend can0 01800001#8FFF7FFF028F3333` |
| **设机械零位** | Type 6 | `set_zero_motor(iface, mid)` | `cansend can0 0600FD01#0100000000000000` |
| **改 CAN ID** | Type 7 | `set_can_id(iface, new_id, current_id)` | `cansend can0 0701FD7F#0000000000000000` |

#### 参数读写

| 操作 | 类型 | Python API | cansend 示例 |
|:-----|:----:|:-----------|:-------------|
| **读参数** | Type 17 | `read_param(iface, index, mid)` | `cansend can0 2200FD01#2230000000000000` |
| **写参数** | Type 18 | `write_param(iface, index, value_bytes, mid)` | `cansend can0 2400FD01#0B7000000000A040` |
| **保存到 flash** | Type 22 | `cansend` only | `cansend can0 2C00FD01#0102030405060708` |

#### 模式切换

| 操作 | 类型 | Python API | 说明 |
|:-----|:----:|:-----------|:------|
| **设运行模式** | Type 18 | `set_mode(iface, mid, mode=0)` | 0=运控, 2=速度, **5=CSP** |
| **主动上报开** | Type 24 | `enable_auto_report(iface, mid)` | 固件 ≥0.0.3.0 可用 |
| **主动上报关** | Type 24 | `disable_auto_report(iface, mid)` | — |

#### 常用参数索引 (Type 17/18)

| 索引 | 名称 | 类型 | 说明 |
|:----:|:-----|:----:|:------|
| 0x200A | CAN_ID | uint8 | 电机 ID (1-127) |
| **0x3022** | **故障码** | uint32 | 读: bit0=过温, bit1=驱动故障, bit2=欠压, bit14=堵转 |
| **0x7005** | **run_mode** | uint8 | 写: 0=运控, 2=速度, 5=CSP |
| **0x700B** | **limit_torque** | float | 力矩限制 (0~14 N.m) |
| **0x7016** | **loc_ref** | float | CSP 位置指令 (rad) |
| **0x7017** | **limit_spd** | float | CSP 速度限制 (0~33 rad/s) |
| 0x7019 | mechPos | float | 当前角度 (rad, 只读) |
| 0x701B | mechVel | float | 当前转速 (rad/s, 只读) |
| 0x701C | VBUS | float | 母线电压 (V, 只读) |
| 0x701E | loc_kp | float | 位置环 Kp (默认 40) |
| 0x701F | spd_kp | float | 速度环 Kp (默认 6) |
| 0x7020 | spd_ki | float | 速度环 Ki (默认 0.02) |
| 0x7022 | acc_rad | float | 加速度 (rad/s²，默认 20) |
| **0x7028** | **canTimeout** | uint32 | CAN 超时 (20000=1s) |
| **0x702B** | **add_offset** | float | 零位偏置 (-7~+7 rad) |

#### 读取故障码

```python
from rs00_control import read_param
read_param('can0', 0x3022, motor_id=1)
# 应答在 monitor 或 candump 中查看 Byte4~7
```

故障码位含义：

| 位 | 含义 |
|:--:|:------|
| bit0 | 电机过温 >145°C |
| bit1 | 驱动芯片故障 |
| bit2 | 欠压 <12V |
| bit3 | 过压 >60V |
| bit7 | 编码器未标定 |
| bit14 | 堵转过载 |

#### CSP 模式完整序列

```python
# 切 CSP (Type 18 写 0x7005=5)
arm.enable_csp(speed_limit=3)

# 设目标 (Type 18 写 0x7016=弧度)
arm.set_angles_csp(90, -45)

# 切回运控 (Type 18 写 0x7005=0)
arm.enable_operation()
```

#### 力矩/速度限制

```python
from rs00_control import write_param
import struct

# 限制最大力矩 8 N.m (0x700B)
write_param('can0', 0x700B, struct.pack('<f', 8.0), motor_id=1)

# 限制 CSP 速度 2 rad/s (0x7017)
write_param('can0', 0x7017, struct.pack('<f', 2.0), motor_id=1)

# CAN 超时 500ms (0x7028)
write_param('can0', 0x7028, struct.pack('<I', 10000), motor_id=1)
```

### 6.4 电机参数

| 参数 | 肩膀 (ID=1) | 肘部 (ID=2) |
|:-----|:----------:|:----------:|
| CAN ID | 1 | 2 |
| 型号 | RS00 14N.m | RS00 14N.m |
| 供电 | 48V | 48V |
| 减速比 | 10:1 | 10:1 |
| 运控 Kp 推荐 | 150 | 50 |
| 用户限位（标零后） | -85° ~ +85° | -70° ~ +110° |
| 电机原始空间 | 170° ~ 340° | 待实测 |

---

## 7. 全 CSP 位置模式工作流（推荐）

### 7.1 为什么放弃运控模式

| 问题 | 运控模式 (Type 1) | CSP 模式 |
|:-----|:-----------------|:---------|
| **调参** | 每次调 Kp/Kd，肩膀需 Kp=150 扛自重 | **不调参**，内部位置环 Kp=40 |
| **到位精度** | PD 力矩控制 → 重力稳态误差 | **0.1° 级精度** |
| **扛自重** | Kp 不够就走不动 | **自动扛** |
| **掰动** | 能掰动 | **掰不动** |
| **速度控制** | Kd 间接控制 | `speed_limit` 参数直接设 |

### 7.2 流程对比

```
运控: enable → set_zero → set_angles(90, -45, kp=150, kd=1).verify()
CSP:  enable → set_zero → enable_csp(1) → set_angles_csp(90, -45).verify()
```

CSP 只多一步 `enable_csp()`，但省去所有 Kp/Kd 调参。

### 7.3 一次性硬件标零

手动把臂摆到机械中位，写入电机 flash，**永久生效**：

```bash
cansend can0 0600FD01#0100000000000000   # 肩膀#1 设为机械零位
cansend can0 0600FD02#0100000000000000   # 肘部#2 设为机械零位
```

Type 6 存储在电机内部，断电不丢失。一生只需做一次。

### 7.4 每天的操作

```python
from rs00_arm import RS00Arm

arm = RS00Arm()
arm.enable()
arm.set_zero()                # 软件标零（当前位置→0°）
arm.enable_csp(speed_limit=1) # 切位置模式，1 rad/s ≈ 57°/s

# 走固定点
arm.set_angles_csp(45, 20).verify()
arm.set_angles_csp(0, 0).verify()
arm.set_angles_csp(-45, -30).verify()

# 动态调速（不重新切模式）
from rs00_control import write_param
import struct
write_param('can0', 0x7017, struct.pack('<f', 3.0), 1)  # 肩膀 3 rad/s
write_param('can0', 0x7017, struct.pack('<f', 3.0), 2)  # 肘部

arm.disable()
```

### 7.5 单电机版本

```python
from rs00_arm import RS00Arm

arm = RS00Arm(elbow_id=None)
arm.enable()
arm.set_zero()
arm.enable_csp(speed_limit=2)

arm.set_angles_csp(45, 0).verify()
arm.set_angles_csp(0, 0).verify()
arm.set_angles_csp(-45, 0).verify()

arm.disable()
```

### 7.6 调速参数速查

| speed_limit | 角速度 | 感觉 |
|:-----------|:------|:------|
| 0.3 | 17°/s | 极慢，精确定位 |
| 0.5 | 28°/s | 很慢，安全测试 |
| **1.0** | **57°/s** | **推荐日常** |
| 2.0 | 114°/s | 较快 |
| 3.0 | 172°/s | 默认（旧版），偏快 |
| 5.0 | 286°/s | 快速 |

运行时动态改（不重新切模式）：

```python
write_param('can0', 0x7017, struct.pack('<f', 1.0), motor_id=1)
```

---

## 8. 注意事项

### 7.1 安全规则

| 规则 | 说明 |
|:-----|:------|
| **先 enable 再发指令** | 不使能电机不出力 |
| **肩膀用高 Kp** | 推荐 150，否则扛不住自重 |
| **先标零再操作** | 每次上电后先 set_zero() |
| **失能前先归零** | 避免下次上电意外动作 |
| **CSP 注意速度** | speed_limit=3 ≈ 172°/s，如需更快调大 |

### 7.2 操作禁区

| 禁止 | 原因 |
|:-----|:------|
| 电机使能时发 `cansend` 直接控制 | 绕过软限位，可能撞坏结构 |
| 使能时用力掰轴 | 可能损坏减速器或编码器 |
| 运行中直接拔 CAN 线 | 可能引起电机乱转 |
| 长时间堵转（>30 秒） | 可能触发过温保护 |

### 7.3 常见问题

| 现象 | 原因 | 解决 |
|:-----|:------|:------|
| cansend: error 105 | CAN 接口没 UP | `sudo ip link set can0 up` |
| 使能后无咔声 | 设备路径不对 | `ls /dev/ttyACM*` 确认 |
| 监控显示 `---` | 还没收到帧 | 发一条指令触发 |
| 肩膀不动 | Kp 太小 | 用 kp=150 或切 CSP |
| 肩膀到位误差大 | 运控模式 + 重力 | 切 CSP 消除稳态误差 |
| monitor 干扰控制 | 旧版心跳冲突 | 已修复，现在是纯监听 |
| 角度超限但电机还在走 | 走的 CSP 路径，旧版无限制 | 已修复，v2.0 加了限位 |

### 7.4 良好习惯

- 单电机用 `RS00Arm(elbow_id=None)` 避免肘部超时
- 每次 `set_angles` 后跟 `.verify()` 确认到位
- 长时间不操作时 `disable()` 释放关节
- 定期检查电机温度（正常 < 50°C，>70°C 需停机）
