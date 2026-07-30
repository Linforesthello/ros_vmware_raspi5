# RS00 全 CSP 位置模式工作流

> 决策记录: 2026-07-10
> 放弃运控模式 (Type 1)，全部走 CSP 位置模式
> **速度设定：`speed_limit=1.0`（57°/s）推荐日常**

---

## 为什么放弃运控模式

| 问题 | 运控模式 (Type 1) | CSP 模式 |
|:-----|:-----------------|:---------|
| **调参** | 每次调 Kp/Kd，肩膀需 Kp=150 扛自重 | **不调参**，电机内部位置环 Kp=40 |
| **到位精度** | PD 力矩控制 → 有重力稳态误差 | **0.1° 级精度**，已验证 |
| **扛自重** | Kp 不够就走不动 | **自动扛** |
| **掰动** | 能掰动 | **掰不动** |
| **速度控制** | Kd 间接控制 | **speed_limit 参数直接设** |

---

## 流程对比

```
运控: enable → set_zero → set_angles(90, -45, kp=150, kd=1).verify()
CSP:  enable → set_zero → enable_csp(1) → set_angles_csp(90, -45).verify()
```

CSP 只多一步 `enable_csp()`，但省去 Kp/Kd 调参。

---

## 全 CSP 工作流

### 第 1 步：一次性硬件标零

手动把臂摆到机械中位，写入电机 flash，**永久生效**：

```bash
cansend can0 0600FD01#0100000000000000   # 肩膀#1 设为机械零位
cansend can0 0600FD02#0100000000000000   # 肘部#2 设为机械零位
```

Type 6 标零后存储在电机内部，断电不丢失。
除非失控爆起冲乱，否则一生只需做一次。

### 第 2 步：日常操作

```bash
cd ~/Lin_workspace/Lin_rs00
python3
```

```python
from rs00_arm import RS00Arm

arm = RS00Arm()
arm.enable()
arm.set_zero()                # 软件标零（当前位置→0°）
arm.enable_csp(speed_limit=1) # 切位置模式，1 rad/s ≈ 57°/s

# 走固定点序列
arm.set_angles_csp(45, 20).verify()
arm.set_angles_csp(0, 0).verify()
arm.set_angles_csp(-45, -30).verify()

# 调速（不重新切模式，运行时改）
from rs00_control import write_param
import struct
write_param('can0', 0x7017, struct.pack('<f', 3.0), 1)  # 肩膀 3 rad/s
write_param('can0', 0x7017, struct.pack('<f', 3.0), 2)  # 肘部 3 rad/s

arm.disable()
```

### 单电机版本

```python
from rs00_arm import RS00Arm

arm = RS00Arm(elbow_id=None)
arm.enable()
arm.set_zero()
arm.enable_csp(speed_limit=1)

arm.set_angles_csp(45, 0).verify()
arm.set_angles_csp(0, 0).verify()
arm.set_angles_csp(-45, 0).verify()

arm.disable()
```

---

## 调速参数速查

| speed_limit | 角速度 | 感觉 |
|:-----------|:------|:------|
| 0.3 | 17°/s | 极慢，精确定位 |
| 0.5 | 28°/s | 很慢，安全测试 |
| **1.0** | **57°/s** | **推荐日常** |
| 2.0 | 114°/s | 较快 |
| 3.0 | 172°/s | 旧版默认，偏快 |
| 5.0 | 286°/s | 快速 |

运行时动态改（不重新切模式）：

```python
write_param('can0', 0x7017, struct.pack('<f', 1.0), motor_id=1)
```

默认值已改为 `speed_limit=1.0`，即 `enable_csp()` 不传参默认 57°/s。

---

## 软件限位（标零后用户空间）

| 关节 | 范围 |
|:-----|:-----|
| 肩膀 | **-85° ~ +85°**（电机原始 170°~340°，中位 255°） |
| 肘部 | **-70° ~ +110°** |

`set_angles_csp()` 已内置限位检查和标零偏移，超限自动钳位 + 警告。

---

## 注意事项

| 注意 | 说明 |
|:-----|:------|
| 失控爆起后检查零位 | 失控可能冲乱 Type 6 机械零位 |
| 恢复方法 | 重新发 Type 6 标零命令 |
| 硬件标零一生一次 | 正常使用后不需要重复做 |
| 每次上电软件标零 | `set_zero()` 每次都要调，否则角度基准不对 |
| 不要连续快速发 CSP | 等 `.verify()` 确认到位后再发下一帧，减少 CAN 总线压力 |
