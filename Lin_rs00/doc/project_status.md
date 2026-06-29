# RS00 双电机机械臂 — 项目状态记录

> 归档日期: 2026-06-28
> 位置: Lin_rs00/

---

## 1. 硬件配置

| 项目 | 配置 |
|------|------|
| 电机 #1 (肩膀) | CAN ID = **1**，特征字节 `48 99` |
| 电机 #2 (肘部) | CAN ID = **2**，特征字节 `2B 19` |
| 主机 ID | **0xFD** (253) |
| CAN 接口 | **can0** (CANable2, slcan 1Mbps) |
| 设备路径 | `/dev/ttyACM0` 或 `/dev/ttyACM1` |
| 协议 | 私有协议 (CAN 扩展帧 29-bit) |

---

## 2. 文件清单

| 文件 | 行数 | 说明 |
|------|:----:|------|
| `rs00_control.py` | 434 | 底层 CAN 指令：使能/停止/运控/改ID/读写参数/读状态 |
| `rs00_arm.py` | 199 | 双电机控制类：链式调用/上下文管理/状态读取 |
| `RS00_速查卡.md` | 149 | 指令速查表 (已更新 Type 7/双电机/角度制) |
| `RS00_新手教程.md` | 356 | 完整教程 (已更新双电机章节/改ID/参数表) |

---

## 3. API 速查

### 底层函数 (rs00_control.py)

```python
# 基础控制
motor_enable(iface, motor_id=127, master_id=0xFD)
motor_disable(iface, motor_id=127, master_id=0xFD)
motor_control(iface, motor_id, pos=°, vel=°/s, kp=, kd=, torque=)
set_mode(iface, motor_id, mode)     # 0=运控 2=速度 5=CSP

# 配置
set_can_id(iface, new_id, master_id=0xFD, current_id=127)
write_param(iface, index, value_bytes, motor_id, master_id)
read_param(iface, motor_id, index=0x3022)

# 状态读取 (同一 socket 收发，零丢帧)
get_motor_state(iface, motor_id, timeout=0.15)
  # → {"position": °, "velocity": °/s, "torque": N.m, "temperature": °C, "motor_id": id}

parse_feedback(data_bytes)  # 解析 Type 2 应答帧

# 设备
setup_can(device="/dev/ttyACM1", baud="s8", interface="can0")
select_device() → device_path
```

### 控制类 (rs00_arm.py)

```python
arm = RS00Arm(iface="can0", shoulder_id=1, elbow_id=2, master_id=0xFD)

# 控制
arm.enable()                           # 同时使能
arm.disable()                          # 同时停止
arm.set_angles(肩°, 肘°, kp=, kd=)    # 设两个角度
arm.set_shoulder(°)                    # 单独肩膀
arm.set_elbow(°)                       # 单独肘部
arm.home(vel=30)                       # 归零 (0°, 0°)

# 状态
arm.get_state()
  # → {"shoulder": {...}, "elbow": {...}}

# 链式
arm.enable().set_angles(90,0).sleep(2).home().disable()

# 上下文
with RS00Arm() as arm:
    arm.set_angles(90, -45).sleep(2)
```

---

## 4. 已验证的功能

### 4.1 通信

| 功能 | 结果 |
|------|:----:|
| 单电机使能/停止/运控 | ✅ |
| 双电机同总线独立控制 | ✅ |
| CAN ID 修改 (Type 7) | ✅ |
| 角度制接口 (°) | ✅ |
| 设备自动检测 | ✅ |

### 4.2 状态读取 (新增)

| 功能 | 结果 |
|------|:----:|
| 实时读角度/速度/力矩/温度 | ✅ |
| 连续 5 次读取成功率 | 100% (修复后) |
| 位置精度 (Kp=10) | 误差 < 1° |

### 4.3 控制类

| 功能 | 结果 |
|------|:----:|
| 链式调用 | ✅ |
| 上下文管理器 (自动使能/释放) | ✅ |
| 双电机同时定位 | ✅ |

---

## 5. 关键技术细节

### 5.1 CAN ID 结构 (私有协议)

```
bit28~24: 通信类型 (5bit)
bit23~8 : 数据字段 (16bit)
bit7~0  : 目标电机 CAN ID (8bit)
```

### 5.2 通信类型

| 类型 | 用途 |
|:----:|------|
| 7 | 改 CAN ID |
| 3 | 使能 |
| 4 | 停止 |
| 1 | 运控指令 |
| 17 (0x11) | 读参数 |
| 18 (0x12) | 写参数 |

### 5.3 Type 1 数据区

```
Byte0~1: 目标位置 [0~65535] → ±720°
Byte2~3: 目标速度 [0~65535] → ±1891 °/s
Byte4~5: Kp       [0~65535] → 0 ~ 500
Byte6~7: Kd       [0~65535] → 0 ~ 5
```

控制公式: `t_ref = Kd×(v_set − v_actual) + Kp×(p_set − p_actual) + t_ff`

### 5.4 Type 2 应答帧

```
Byte0~1: 当前角度 [0~65535] → ±720°
Byte2~3: 当前速度 [0~65535] → ±1891 °/s
Byte4~5: 当前力矩 [0~65535] → ±14 N.m
Byte6~7: 温度 (×10)
```

### 5.5 角度换算

```python
P_MIN, P_MAX = -720.0, 720.0       # 位置范围 (°)
V_MIN, V_MAX = -1891.0, 1891.0     # 速度范围 (°/s)
```

---

## 6. 未完成 / 下一步

- [ ] **机械结构** — 两个电机之间的连接支架（肩→肘）
- [ ] **末端夹爪** — 选型/安装
- [ ] **RS00Arm 增强** — 轨迹插补、限位保护
- [ ] **正运动学** — 给定角度 → 末端坐标
- [ ] **逆运动学** — 给定坐标 → 角度
