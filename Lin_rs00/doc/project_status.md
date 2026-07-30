# RS00 双电机机械臂 — 项目状态记录

> 归档日期: 2026-07-10
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
| 供电 | 48 VDC (24-60V) |
| 减速比 | 10:1 |

---

## 2. 文件清单

| 文件 | 行数 | 说明 |
|:------|:----:|:------|
| `rs00_control.py` | 496 | 底层 CAN 指令 + CAN socket 收发 + 状态读取 |
| `rs00_arm.py` | 478 | 双电机控制类 + CSP + 限位 + 标零 + 验证 + 单电机模式 |
| `monitor.py` | 235 | 实时监控（纯被动监听，不发送任何指令） |
| `doc/RS00_速查卡.md` | ~350 | 完整指令集 + API 快速调用合集 |
| `doc/RS00_新手教程.md` | 406 | 从零教程 |
| `doc/usage_guide.md` | ~320 | 操作手册：全流程 + 示例 |
| `doc/project_status.md` | 本文件 | 项目状态归档 |

---

## 3. 完整 API 速查

### 底层 (rs00_control.py)

| 函数 | 对应类型 | 说明 |
|:------|:--------:|:------|
| `motor_enable(iface, motor_id)` | Type 3 | 使能 |
| `motor_disable(iface, motor_id)` | Type 4 | 停止 |
| `motor_control(iface, motor_id, pos, vel, kp, kd, torque)` | Type 1 | 运控指令 |
| `set_zero_motor(iface, motor_id)` | Type 6 | 设机械零位 |
| `set_can_id(iface, new_id, current_id)` | Type 7 | 改 CAN ID |
| `get_motor_state(iface, motor_id, kp_hold, kd_hold, pos_hold, vel_hold)` | Type 1 + 监听 | 读状态 (零干扰) |
| `write_param(iface, index, value_bytes, motor_id)` | Type 18 | 写参数 |
| `read_param(iface, index, motor_id)` | Type 17 | 读参数请求 |
| `set_mode(iface, motor_id, mode)` | Type 18 | 设运行模式 (0=运控, 5=CSP) |
| `enable_auto_report(iface, motor_id, interval_ms)` | Type 24 | 开启主动上报 |
| `disable_auto_report(iface, motor_id)` | Type 24 | 关闭主动上报 |
| `setup_can(device, baud, iface)` | — | 配置 CAN 接口 |
| `select_device()` | — | 自动检测设备 |

### 控制类 (rs00_arm.py)

| 方法 | 说明 |
|:------|:------|
| `enable()` / `disable()` | 使能 / 停止（肘部 None 时跳过） |
| `set_angles(肩°, 肘°, kp=10, kd=1)` | 运控模式设角度 (自动限位) |
| `set_shoulder(°)` / `set_elbow(°)` | 单关节控制 |
| `enable_csp(speed_limit, accel_limit)` | **切换 CSP 高刚度模式** |
| `set_angles_csp(肩°, 肘°)` | **CSP 模式设角度（含限位+标零）** |
| `enable_operation()` | **切回运控模式** |
| `set_zero()` | **标零 (当前位置→0°)** |
| `set_limits(min, max)` | **设置限位** |
| `verify(settle_time, tolerance)` | **验证到位** |
| `get_state()` | 读双电机状态 |
| `home()` / `stop()` | 归零 / 急停 |
| `sleep(秒)` | 等待 (链式支持) |

### 监控 (monitor.py)

```bash
python3 monitor.py                       # 纯被动监听（默认，不干扰控制）
python3 monitor.py --auto-report         # 自动上报（需固件 ≥0.0.3.0）
python3 monitor.py --id 1                # 只监听肩膀
```

---

## 4. 已验证的功能

| 功能 | 结果 |
|:------|:----:|
| 单电机使能/停止/运控 | ✅ |
| 双电机同总线独立控制 | ✅ |
| CAN ID 修改 (Type 7) | ✅ |
| 角度制接口 (°) | ✅ |
| 设备自动检测 | ✅ |
| 参数读写 (Type 17/18) | ✅ |
| 实时状态读取 (零丢帧) | ✅ |
| 无干扰查询 (位置/刚度保持) | ✅ |
| 软件限位 + 超限钳位 | ✅ |
| 标零 (set_zero) | ✅ |
| CSP 高刚度锁定 | ✅ |
| CSP 限位保护 + 标零偏移 | ✅ v2.0 新增 |
| 运控/CSP 模式切换 | ✅ |
| 指令验证 (verify) | ✅ |
| 被动监控 (不干扰控制) | ✅ |
| 单电机模式 (elbow_id=None) | ✅ v2.0 新增 |
| 链式调用 / 上下文管理器 | ✅ |
| 参数动态调速 (CSP speed_limit) | ✅ |

---

## 5. 实测限位（标零后用户空间）

| 关节 | 电机原始范围 | 中位标零后 |
|:-----|:------------|:-----------|
| 肩膀 (ID=1) | **170° ~ 340°** | **-85° ~ +85°** |
| 肘部 (ID=2) | 待实测 | **-70° ~ +110°**（预估） |

---

## 6. 修复记录

| 日期 | 问题 | 原因 | 修复 |
|:----:|:-----|:-----|:-----|
| **2026-07-10** | **monitor 心跳干扰外部控制** | monitor 发 Type 1 指令刷新数据，覆盖用户的控制参数 | **移除心跳发送，monitor 改为纯被动监听** |
| **2026-07-10** | **CSP 绕过限位+标零** | `set_angles_csp()` 直接发弧度，没调 `_clamp_angles()` 和 `_user_to_motor()` | **CSP 加入限位检查和标零偏移** |
| **2026-07-10** | **elbow_id=None 报错** | 所有方法未处理 `None` | **所有方法加 None 判断** |
| **2026-07-10** | **肩膀限位错误** | `DEFAULT_SHOULDER_MIN=-40°`，实际物理可达 -85° | **改为 -85°~+85°** |
| **2026-07-10** | **肩膀 Kp=30 扛不动自重** | 运控模式 Kp 默认 10，肩膀负载大 | **文档明确推荐 Kp=150** |
| 2026-06-29 | **get_state() 串读**：set_zero 肘部零点交替跳 -117° | `get_motor_state` 收到非目标电机的应答帧 | 增加电机 ID 核验 |
| 2026-06-29 | **首次 get_state 拽电机** | `_last_kp=10` 把电机往 0 拽 | `_last_kp` 初始化为 0，加 `pos_hold` |
| 2026-06-29 | **monitor 与控制冲突（旧）** | 主动查询干扰电机 | 改为被动监听 + 智能心跳 |
| 2026-06-28 | **双电机状态连续读取丢帧** | `cansend` 子进程竞争 | 改为同一 socket 收发 |

---

## 7. 已知问题

| 问题 | 状态 | 说明 |
|:-----|:----:|:------|
| Type 24 自动上报 | ⚠️ | 固件 <0.0.3.0 不支持，不影响使用 |
| 运控模式肩膀到位有误差 | ⚠️ | Type 1 本质是 PD 力矩控制，有重力稳态误差。**切 CSP 解决** |
| 肘部物理范围未实测 | ⏳ | 当前用旧值 ±110°，建议实测校准 |

---

## 8. 肘部范围实测记录（2026-07-10）

CSP 模式测试结果：

| 目标角度 | 实际到位 | 结果 |
|:--------|:---------|:----:|
| 0° | -0.1° | ✅ |
| 10° | 9.9° | ✅ |
| 90° | 89.9° | ✅ |
| 110° | 109.9° | ✅（上限） |
| -5° | -5.1° | ✅ |
| -25° | -25.1° | ✅ |
| -50° | -50.1° | ✅ |
| -70° | -70.1° | ✅（下限） |
| -90° | 被钳位到 -70° | ⚠️ 物理不可达 |

**结论**：肘部用户空间范围 **-70° ~ +110°** 已验证，与当前默认值一致，无需修改。

---

## 9. 下一步

- [ ] **肘部物理范围实测** — 确认电机原始空间角度
- [ ] **完整 R1 机械臂联调** — RS00 肩肘 + SteeringArm 腕+夹爪 + 舵轮底盘
