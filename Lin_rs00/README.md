# RS00 双电机机械臂控制

肩肘串联双关节 R1 机械臂的 PC 端 CAN 控制软件。

---

## 🔴 重要度排序（从高到低）

### P0 — 必须掌握

| # | 内容 | 文档 |
|:-:|:-----|:-----|
| 1 | **全 CSP 位置模式工作流**（推荐，放弃运控） | `doc/csp_workflow.md` |
| 2 | **硬件标零**（Type 6，写入 flash 一生一次） | `doc/csp_workflow.md#第1步` |
| 3 | **每天的操作流程** | `doc/usage_guide.md#2-标准操作流程` |

### P1 — 日常参考

| # | 内容 | 文档 |
|:-:|:-----|:-----|
| 4 | **API 快速调用**（复制粘贴） | `doc/RS00_速查卡.md#快速调用合集` |
| 5 | **调速参数速查** | `doc/csp_workflow.md#调速参数速查` |
| 6 | **状态监控** | `doc/usage_guide.md#5-状态监控` |

### P2 — 了解即可

| # | 内容 | 文档 |
|:-:|:-----|:-----|
| 7 | 从零教程 | `doc/RS00_新手教程.md` |
| 8 | 运控模式备用 | `doc/usage_guide.md#3-两种控制模式对比` |
| 9 | CAN 协议细节 | `doc/can_protocol_design.md` |

---

## 快速开始

```bash
cd ~/Lin_workspace/Lin_rs00

# 启动 CAN 接口
CanCmd                           # 选 1) 1Mbps

# 进交互控制
python3
```

```python
from rs00_arm import RS00Arm

arm = RS00Arm(elbow_id=None)     # 单电机模式
arm.enable()
arm.set_zero()                   # 当前位置 → 0°
arm.enable_csp(speed_limit=1)    # 切位置模式，1 rad/s ≈ 57°/s
arm.set_angles_csp(45, 0).verify()
arm.set_angles_csp(0, 0).verify()
arm.disable()
```

---

## 文件结构

```
Lin_rs00/
├── rs00_control.py      # 底层 CAN 指令 (Type 1/3/4/6/7/17/18/24)
├── rs00_arm.py          # 双电机控制类 (运控+CSP+限位+标零+单电机模式)
├── monitor.py           # 实时监控（纯被动监听，不干扰控制）
├── dev_plan.md          # R1 全系统架构与开发计划
├── README.md            # 本文件
└── doc/
    ├── csp_workflow.md       # 🔴 全 CSP 工作流（推荐，首选阅读）
    ├── usage_guide.md        # 🔴 操作手册（完整流程）
    ├── RS00_速查卡.md         # 🟡 API 速查 + 指令合集
    ├── RS00_新手教程.md       # 🟢 从零教程
    ├── project_status.md     # 项目状态与修复记录
    ├── system_architecture.md   # 系统架构设计
    ├── dual_channel_can.md      # 双通道 CAN 架构
    ├── can_protocol_design.md   # CAN 协议设计
    └── hardware_reliability.md  # 硬件可靠性分析
```

---

## 核心用法

### 双电机

```python
from rs00_arm import RS00Arm

arm = RS00Arm()                     # 肩#1 肘#2
arm.enable()
arm.set_zero()
arm.enable_csp(speed_limit=1)       # 1 rad/s ≈ 57°/s

arm.set_angles_csp(45, 20).verify()
arm.set_angles_csp(0, 0).verify()

arm.disable()
```

### 单电机

```python
from rs00_arm import RS00Arm

arm = RS00Arm(elbow_id=None)        # 肘部跳过
arm.enable()
arm.set_zero()
arm.enable_csp(speed_limit=1)

arm.set_angles_csp(45, 0).verify()
arm.set_angles_csp(0, 0).verify()

arm.disable()
```

### 硬件标零（一生一次）

手动把臂摆到机械中位后执行：

```bash
cansend can0 0600FD01#0100000000000000   # 肩膀#1
cansend can0 0600FD02#0100000000000000   # 肘部#2
```

### 运行时调速

```python
from rs00_control import write_param
import struct
write_param('can0', 0x7017, struct.pack('<f', 0.5), motor_id=1)  # 肩膀 0.5 rad/s
write_param('can0', 0x7017, struct.pack('<f', 3.0), motor_id=2)  # 肘部 3.0 rad/s
```

### 调速速查

| speed_limit | 角速度 | 场景 |
|:-----------|:------|:------|
| 0.3 | 17°/s | 极慢，精确定位 |
| 0.5 | 28°/s | 安全测试 |
| **1.0** | **57°/s** | **推荐日常** |
| 2.0 | 114°/s | 较快 |
| 3.0 | 172°/s | 快速 |

---

## 监控

```bash
cd ~/Lin_workspace/Lin_rs00
python3 monitor.py              # 两个电机同时监控
python3 monitor.py --id 1       # 只肩膀
python3 monitor.py --id 2       # 只肘部
```

纯被动监听，不发送任何指令，可与 Python REPL / cansend 同时使用。

---

## 硬件规格

| 项目 | 规格 |
|:-----|:------|
| 电机 | RS00 准直驱，14N.m 峰值，10:1 减速比 |
| CAN | 1Mbps，29-bit 扩展帧 |
| 适配器 | CANable2 (slcan) |
| 供电 | 48 VDC |
| 肩膀限位 | -85°~+85°（电机原始 170°~340°，中位 255°） |
| 肘部限位 | -70°~+110° |

---

## 注意事项

| 规则 | 说明 |
|:-----|:------|
| 先 `enable()` 再发指令 | 不使能电机不出力 |
| 每次上电先 `set_zero()` | 否则角度基准不对 |
| 推荐 CSP > 运控 | CSP 不调参、精度高、掰不动 |
| 失控爆起检查零位 | 可能冲乱 Type 6，重新标零恢复 |
| 不要连续快速发 CSP | 等 `.verify()` 确认到位后再发下一帧 |
