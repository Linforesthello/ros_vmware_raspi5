#!/usr/bin/env python3
"""
舵轮底盘控制 — 树莓派独立版
============================
自包含版本，不需要 mclm_can.py。
直接在树莓派上运行，通过 CANable2 控制 R1 底盘。

用法:
    cd ~/r1_control
    source venv/bin/activate
    python3 pi_chassis_control.py

命令:
    0.5 0 0.1         → vx=0.5 vy=0.0 omega=0.1
    stop              → 全车停止
    s                 → 显示当前状态
    cal               → 全部单元当前位归零
    q                 → 退出
"""

import sys, time, math, threading
import can as python_can
from dataclasses import dataclass
from enum import IntEnum

# ═══════════════════════════════════════════════════════════════
# CAN ID 定义
# ═══════════════════════════════════════════════════════════════

# CAN ID 组（每组对应一个物理轮子，通过 CAN_ID_GROUP 宏在 MCLM 端配置）
#   实测映射 (2026-07-13):
#     UNIT1 (CAN_ID_GROUP=3, 0x121/0x122) → 右后 (Rear-Right)
#     UNIT2 (CAN_ID_GROUP=2, 0x123/0x124) → 右前 (Front-Right)
#     UNIT3 (CAN_ID_GROUP=1, 0x125/0x126) → 左前 (Front-Left)
#     UNIT4 (CAN_ID_GROUP=4, 0x127/0x128) → 左后 (Rear-Left)

UNIT_CAN_IDS = [
    {'turn': 0x121, 'power': 0x122, 'status_turn': 0x321, 'status_power': 0x322},  # UNIT1: 右后
    {'turn': 0x123, 'power': 0x124, 'status_turn': 0x323, 'status_power': 0x324},  # UNIT2: 右前
    {'turn': 0x125, 'power': 0x126, 'status_turn': 0x325, 'status_power': 0x326},  # UNIT3: 左前
    {'turn': 0x127, 'power': 0x128, 'status_turn': 0x327, 'status_power': 0x328},  # UNIT4: 左后
]

CMD_SET_SPEED = 0x11
CMD_STOP      = 0x08

# ═══════════════════════════════════════════════════════════════
# 串口遥控配置（可通过 startup_config() 交互修改）
# ═══════════════════════════════════════════════════════════════
SERIAL_ENABLED = True                     # 是否启动串口监听
SERIAL_PORT = "/dev/ttyACM1"              # 接收机串口（USB转串口）
SERIAL_BAUD = 115200                      # 串口波特率
CAN_BITRATE = 1000000                     # CAN 总线波特率
REMOTE_MODE = False                       # 遥控模式（true时键盘输入禁用）
REMOTE_CH_DEADBAND = 10                   # 摇杆死区(0-100)
# 摇杆通道映射: [通道号0~N] → vx, vy, omega
# 默认: CH0=vx, CH1=vy, CH2=omega, 值范围 -100~100
CH_MAP = {'vx': 0, 'vy': 1, 'omega': 2}

# 遥控指令文本协议（从串口读取一行，按空格分割）
# 格式1(推荐): "0.5 0 0.1"    → vx=0.5 vy=0 omega=0.1
# 格式2(手动): "0.5 0 0.1"   → 同CLI输入

# ═══════════════════════════════════════════════════════════════
# 底盘参数
# ═══════════════════════════════════════════════════════════════
# 轮子位置 (x, y) — x⁺=前进方向, y⁺=左侧
WHEEL_POS = [
    (-0.25, -0.30),   # UNIT1: 右后  (0x121)
    ( 0.25, -0.30),   # UNIT2: 右前  (0x123)
    ( 0.25,  0.30),   # UNIT3: 左前  (0x125)
    (-0.25,  0.30),   # UNIT4: 左后  (0x127)
]

# 编码器参数
TICKS_PER_REV = [9218, 9196, 7844, 9395]  # 实测 (2026-07-14)
KP_ANGLE = 1.0        # 比例增益
KI_ANGLE = 0.03       # 积分增益（消除静差）
MAX_STEER_SPEED = 60
DEAD_ZONE_DEG = 0.5       # 转向死区
STEER_READY_DEG = 5.0     # 舵向就绪阈值：所有轮子误差<此值才允许驱动
STEER_RAMP_RATE = 8

# 全局状态
prev_err = [0.0, 0.0, 0.0, 0.0]   # 上次误差（D 项用）
integral = [0.0, 0.0, 0.0, 0.0]   # 积分累计
INTEGRAL_LIMIT = 300               # 积分限幅，防 windup       # 每周期(50ms)转向速度最大变化量，减缓冲击

# ═══════════════════════════════════════════════════════════════
# 保护参数
# ═══════════════════════════════════════════════════════════════
STALL_TIMEOUT = 2.0        # 堵转超过此秒数 → 自动切断电源
PROTECTION_ACTIVE = [False, False, False, False]  # 保护是否已触发
stall_accum = [[0.0, 0.0] for _ in range(4)]     # [unit][0=turn, 1=power] 累计堵转秒数
DISABLED_UNITS = set()     # 禁用的单元号（0-based），如 {3} = 跳过 UNIT4

# 当前 CAN 帧中的 flags 缓存（供 print_status 和保护判断用）
last_flags = [{'turn': 0, 'power': 0} for _ in range(4)]
stall_warned = [False, False, False, False]  # 堵转告警去重
drive_ready = True                        # 舵向是否就绪（允许驱动）
cur_raw_ticks = [0, 0, 0, 0]
abs_ticks = [0, 0, 0, 0]
prev_raw = [None, None, None, None]
cur_status_time = [0, 0, 0, 0]
target_steer_deg = [0.0, 0.0, 0.0, 0.0]
target_drive_speed = [0, 0, 0, 0]
angle_mode = False
last_sent_steer = [999, 999, 999, 999]
last_sent_drive = [999, 999, 999, 999]
ZERO_OFFSETS = [0, 0, 0, 0]
running = True

# CAN 总线
can_bus = None

# 遥控超时保护
REMOTE_TIMEOUT = 1.0        # 遥控数据超过此秒数未更新 → 全车停止
last_remote_time = 0.0      # 上次收到遥控数据的时间戳



def can_send(can_id: int, data: bytes):
    """发送 CAN 帧"""
    if can_bus:
        msg = python_can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        try:
            can_bus.send(msg)
        except Exception as e:
            print(f"[CAN TX ERR] 0x{can_id:X}: {e}")


def stop_all():
    """全车停止"""
    can_send(0x101, bytes([CMD_STOP, 0, 0, 0, 0, 0, 0, 0]))
    for u in UNIT_CAN_IDS:
        can_send(u['turn'], bytes([CMD_STOP, 0, 0, 0, 0, 0, 0, 0]))
        can_send(u['power'], bytes([CMD_STOP, 0, 0, 0, 0, 0, 0, 0]))


def reset_protection():
    """解除保护状态（手动确认故障已排除后调用）"""
    global PROTECTION_ACTIVE, stall_accum, stall_warned, prev_err, integral
    for u in range(4):
        PROTECTION_ACTIVE[u] = False
        stall_accum[u] = [0.0, 0.0]
        stall_warned[u] = False
        prev_err[u] = 0.0
        integral[u] = 0.0
    print(">> 保护已重置")


# ── 串口遥控 ──
serial_remote_vx = 0.0
serial_remote_vy = 0.0
serial_remote_omega = 0.0
serial_remote_updated = False
serial_remote_lock = threading.Lock()

# 串口文本命令（cal/stop/steer 等），由 serial_reader 写入
remote_cmd = None
remote_cmd_lock = threading.Lock()

def serial_reader():
    """串口遥控读取线程"""
    global serial_remote_vx, serial_remote_vy, serial_remote_omega, serial_remote_updated
    import serial as serial_lib
    ser = None
    last_printed = ""
    while running:
        try:
            if ser is None or not ser.is_open:
                ser = serial_lib.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
                print(f"[遥控] ✅ 串口已打开: {SERIAL_PORT} @ {SERIAL_BAUD}")
        except Exception as e:
            print(f"[遥控] ⚠️ 串口打开失败: {e} (2秒后重试)")
            time.sleep(2)
            continue
        try:
            line = ser.readline().decode(errors='ignore').strip()
            if not line:
                continue
            parts = line.split()
            # 去重打印：同一个值只打印一次
            if line != last_printed:
                print(f"[遥控] 📩 {repr(line)}")
                last_printed = line
            if len(parts) >= 3:
                try:
                    vx = float(parts[0])
                    vy = float(parts[1])
                    omega = float(parts[2])
                    with serial_remote_lock:
                        serial_remote_vx = vx
                        serial_remote_vy = vy
                        serial_remote_omega = omega
                        serial_remote_updated = True
                    print(f"[遥控] ✅ vx={vx:.2f} vy={vy:.2f} ω={omega:.2f}")
                except ValueError as e:
                    print(f"[遥控] ⚠️ 解析失败: {e}")
        except Exception as e:
            print(f"[遥控] ⚠️ 读取错误: {e}")
            try:
                ser.close()
            except Exception:
                pass
            ser = None
            time.sleep(1)


def apply_remote_command():
    """将串口遥控的最新值应用到目标"""
    global target_steer_deg, target_drive_speed, angle_mode, serial_remote_vx, serial_remote_vy, serial_remote_omega, serial_remote_updated, last_remote_time
    if not SERIAL_ENABLED or not REMOTE_MODE:
        return
    with serial_remote_lock:
        if not serial_remote_updated:
            return
        vx = serial_remote_vx
        vy = serial_remote_vy
        omega = serial_remote_omega
        serial_remote_updated = False
        last_remote_time = time.time()

    print(f"[遥控] ▶ 执行: vx={vx:.2f} vy={vy:.2f} ω={omega:.2f}")

    # 安全检查：限制最大速度
    vx = max(-1.0, min(1.0, vx))
    vy = max(-1.0, min(1.0, vy))
    omega = max(-1.0, min(1.0, omega))

    results = inv_kinematics(vx, vy, omega)
    MAX_MPS = 0.8
    for u, (steer, drive) in enumerate(results):
        target_steer_deg[u] = steer % 360
        drive_logic = int(drive / MAX_MPS * 100)
        drive_logic = max(-100, min(100, drive_logic))
        if abs(vx) < 0.01 and abs(vy) < 0.01 and abs(omega) > 0.01:
            if 0 < drive_logic < 20:
                drive_logic = 20
        target_drive_speed[u] = drive_logic
    angle_mode = True


def set_speed(unit_idx: int, motor_type: str, speed: int):
    """设置单个电机速度"""
    speed = max(-100, min(100, speed))
    can_id = UNIT_CAN_IDS[unit_idx]['turn'] if motor_type == 'turn' else UNIT_CAN_IDS[unit_idx]['power']
    can_send(can_id, bytes([CMD_SET_SPEED, speed & 0xFF, 0, 0, 0, 0, 0, 0]))


def unwrap(u, raw):
    """uint16 → 连续 int32（回绕防护）"""
    global abs_ticks, prev_raw
    if prev_raw[u] is None:
        abs_ticks[u] = raw
    else:
        diff = raw - prev_raw[u]
        if diff > 32768:
            diff -= 65536
        elif diff < -32768:
            diff += 65536
        abs_ticks[u] += diff
    prev_raw[u] = raw


def angle_now(u):
    """当前转向角度（度），基于连续 ticks"""
    return ((abs_ticks[u] - ZERO_OFFSETS[u]) % TICKS_PER_REV[u]) / TICKS_PER_REV[u] * 360.0


def shortest_error(current, target):
    return (target - current + 180) % 360 - 180


def inv_kinematics(vx, vy, omega):
    """
    运动学逆解: 车体速度 → 各单元转向角度 + 驱动速度

    运动学约定: +角度 = 逆时针(向左转)
    电机物理:  +speed = 顺时针(向右转)
    因此: steer_deg 取反，使角度指令与电机方向对齐
    """
    results = []
    for x, y in WHEEL_POS:
        v_ix = vx - omega * y
        v_iy = vy + omega * x
        steer = -math.degrees(math.atan2(v_iy, v_ix))
        drive = math.hypot(v_ix, v_iy)
        results.append((steer, drive))
    return results


def print_status():
    global drive_ready
    print("-" * 85)
    print(f"  单元    转向角度  目标  驱动  flags  保护  状态")
    print("-" * 85)
    for u in range(4):
        angle = angle_now(u)
        err = shortest_error(angle, target_steer_deg[u]) if angle_mode else 0
        if u in DISABLED_UNITS:
            print(f"  UNIT{u+1}        ❌ 已禁用")
            continue
        alive = (time.time() - cur_status_time[u]) < 2.0
        if alive:
            flag = '✓' if abs(err) < DEAD_ZONE_DEG else '→'
            tf = last_flags[u]['turn']
            pf = last_flags[u]['power']
            prot = "🔒" if PROTECTION_ACTIVE[u] else "  "
            print(f"  UNIT{u+1}  {angle:7.1f}°  {target_steer_deg[u]:4.0f}°  "
                  f"{target_drive_speed[u]:+4d}  "
                  f"T=0x{tf:02X} P=0x{pf:02X}  {prot}  {flag}")
        else:
            print(f"  UNIT{u+1}        离线")
    drive_status = "🟢 可驱动" if drive_ready else "🔴 等待舵向就绪"
    print(f"  {drive_status}")
    print("-" * 85)


def cli_thread():
    global angle_mode, target_steer_deg, target_drive_speed, ZERO_OFFSETS, running
    global PROTECTION_ACTIVE, stall_accum, stall_warned
    global last_sent_steer, last_sent_drive
    global prev_err, integral
    global REMOTE_MODE, serial_remote_updated, DISABLED_UNITS, last_remote_time
    while running:
        try:
            line = input().strip().lower()
        except EOFError:
            break
        if not line:
            continue
        if line == 'q':
            running = False
            break
        elif line == 'stop':
            # 关键：先发 CAN 急停让电机物理停止，再关控制循环
            for u in range(4):
                if u not in DISABLED_UNITS:
                    set_speed(u, 'turn', 0)
                    set_speed(u, 'power', 0)
            target_drive_speed = [0, 0, 0, 0]
            last_sent_steer = [999, 999, 999, 999]
            last_sent_drive = [999, 999, 999, 999]
            angle_mode = False  # 关掉控制循环，不让 PID 再发命令
            reset_protection()
            print(">> 全车停止（控制循环已关闭）")
        elif line == 'reset':
            reset_protection()
        elif line == 'remote':
            REMOTE_MODE = not REMOTE_MODE
            if REMOTE_MODE:
                last_remote_time = time.time()  # 重置超时计时
                print(">> 📡 遥控模式已开启（串口 → 底盘）")
            else:
                print(">> ⌨️  键盘模式已恢复")
                # 切回键盘时停止
                for u in range(4):
                    set_speed(u, 'turn', 0)
                    set_speed(u, 'power', 0)
                target_drive_speed = [0, 0, 0, 0]
                angle_mode = False
        elif line.startswith('disable '):
            try:
                u = int(line.split()[1]) - 1  # 1-based → 0-based
                if 0 <= u < 4:
                    DISABLED_UNITS.add(u)
                    set_speed(u, 'turn', 0)
                    set_speed(u, 'power', 0)
                    print(f">> UNIT{u+1} 已禁用")
                else:
                    print(">> 单元号 1~4")
            except (IndexError, ValueError):
                print(">> disable <单元号 1-4>")
        elif line.startswith('enable '):
            try:
                u = int(line.split()[1]) - 1
                DISABLED_UNITS.discard(u)
                print(f">> UNIT{u+1} 已启用")
            except (IndexError, ValueError):
                print(">> enable <单元号 1-4>")
        elif line == 's':
            print_status()
        elif line.startswith('steer '):
            try:
                sa = float(line.split()[1]) % 360
            except (IndexError, ValueError):
                print(">> steer <角度>")
                continue
            for u in range(4):
                # steer 命令输入为正角度(CCW/向左)，但电机 +speed=CW
                # 取反让目标与电机方向一致
                target_steer_deg[u] = (-sa) % 360.0
                target_drive_speed[u] = 0
            angle_mode = True
            print(f">> 全部舵向→{sa:.0f}°（CCW/左为正），不动驱动轮")
        elif line == 'cal':
            # 检查是否收到 CAN 帧（跳过已禁用单元）
            offline = [u+1 for u in range(4) if u not in DISABLED_UNITS and (time.time() - cur_status_time[u]) > 2.0]
            if offline:
                print(f">> ❌ UNIT{offline} 未收到 CAN 状态帧，无法归零")
                continue
            # 先停电机 + 清保护
            for u in range(4):
                if u not in DISABLED_UNITS:
                    set_speed(u, 'turn', 0)
                    set_speed(u, 'power', 0)
            reset_protection()
            time.sleep(0.2)
            for u in range(4):
                ZERO_OFFSETS[u] = abs_ticks[u]
                target_steer_deg[u] = 0.0
                last_sent_steer[u] = 999
                last_sent_drive[u] = 999
            angle_mode = True
            disabled = [str(u+1) for u in DISABLED_UNITS]
            status = "已禁用: UNIT" + ", ".join(disabled) if disabled else ""
            print(f">> ✅ 全部单元已归零  {status}")
            print(">> 当前角度被设为 0° — 如果轮子不是朝正前方，必须手动掰正后重新 cal")
        elif line.startswith('test '):
            parts = line.split()
            try:
                tvx = float(parts[1]) if len(parts) > 1 else 0
                tvy = float(parts[2]) if len(parts) > 2 else 0
                tom = float(parts[3]) if len(parts) > 3 else 0
            except ValueError:
                print(">> test vx vy omega")
                continue
            tresults = inv_kinematics(tvx, tvy, tom)
            print(f">> [公式预览] vx={tvx:.2f} vy={tvy:.2f} omega={tom:.2f}")
            for u, (st, dr) in enumerate(tresults):
                print(f"     UNIT{u+1}: 角度={st:7.1f}°  驱动速度={dr:.3f}")
        else:
            parts = line.split()
            try:
                vx = float(parts[0])
                vy = float(parts[1]) if len(parts) > 1 else 0
                omega = float(parts[2]) if len(parts) > 2 else 0
            except ValueError:
                print(">> 格式: vx vy omega")
                continue

            results = inv_kinematics(vx, vy, omega)
            MAX_MPS = 0.8
            for u, (steer, drive) in enumerate(results):
                target_steer_deg[u] = steer % 360
                drive_logic = int(drive / MAX_MPS * 100)
                drive_logic = max(-100, min(100, drive_logic))
                if abs(vx) < 0.01 and abs(vy) < 0.01 and abs(omega) > 0.01:
                    if 0 < drive_logic < 20:
                        drive_logic = 20
                target_drive_speed[u] = drive_logic
            angle_mode = True
            print(f">> vx={vx:.2f} vy={vy:.2f} omega={omega:.2f}")


def startup_config():
    """启动交互配置：串口、波特率、CAN 波特率"""
    global SERIAL_PORT, SERIAL_BAUD, CAN_BITRATE, SERIAL_ENABLED
    import glob
    print("=" * 50)
    print("  R1 底盘控制 — 启动配置")
    print("=" * 50)

    # 列出可用串口
    ports = []
    for p in sorted(glob.glob('/dev/ttyACM*')): ports.append(p)
    for p in sorted(glob.glob('/dev/ttyUSB*')): ports.append(p)
    for p in sorted(glob.glob('/dev/serial/by-id/*')):
        import os
        real = os.path.realpath(p)
        if real not in ports: ports.append(p)

    print("\n可用串口:")
    if ports:
        for i, p in enumerate(ports):
            print(f"  [{i}] {p}")
        try:
            idx = int(input(f"选择遥控串口 [0-{len(ports)-1}], 回车跳过不开启: "))
            if 0 <= idx < len(ports):
                SERIAL_PORT = ports[idx]
                print(f"  → 串口: {SERIAL_PORT}")
        except (ValueError, IndexError):
            pass
    else:
        print("  (无串口设备)")
        SERIAL_ENABLED = False

    if SERIAL_ENABLED:
        b = input(f"串口波特率 [默认{SERIAL_BAUD}]: ").strip()
        if b: SERIAL_BAUD = int(b)
        print(f"  → 波特率: {SERIAL_BAUD}")

    c = input(f"CAN 波特率 [默认1000000=1Mbps]: ").strip()
    if c: CAN_BITRATE = int(c)
    print(f"  → CAN: {CAN_BITRATE}bps")
    print()


def main():
    global can_bus, running, drive_ready, DISABLED_UNITS, last_remote_time, angle_mode, target_drive_speed, REMOTE_MODE
    global SERIAL_PORT, SERIAL_BAUD, CAN_BITRATE, SERIAL_ENABLED
    global remote_cmd, remote_cmd_lock
    global last_sent_steer, last_sent_drive

    # 启动配置
    startup_config()

    # 打开 CAN 总线
    print(f"正在连接 can0 ({CAN_BITRATE}bps)...")
    try:
        can_bus = python_can.Bus(channel='can0', bustype='socketcan', bitrate=CAN_BITRATE)
    except Exception as e:
        print(f"[ERROR] CAN 打开失败: {e}")
        return

    # 状态帧接收线程
    prev_rx_time = [0.0, 0.0, 0.0, 0.0]  # 上一帧收到时间（用于 stall 累加）

    def rx_loop():
        nonlocal prev_rx_time
        while running:
            try:
                msg = can_bus.recv(timeout=0.1)
                if msg is None:
                    continue
                can_id = msg.arbitration_id
                data = msg.data
                if len(data) < 8:
                    continue
                for u, ids in enumerate(UNIT_CAN_IDS):
                    now = time.time()
                    if can_id == ids['status_turn']:
                        # 转向电机状态帧 → 更新角度 + flags
                        t = data[2] | (data[3] << 8)
                        cur_raw_ticks[u] = t
                        unwrap(u, t)
                        cur_status_time[u] = now
                        flags = data[7]
                        last_flags[u]['turn'] = flags
                        # 堵转计时
                        dt = now - prev_rx_time[u]
                        if flags & 0x01:  # STALL
                            stall_accum[u][0] += dt
                        else:
                            stall_accum[u][0] = 0.0
                        prev_rx_time[u] = now
                        break
                    elif can_id == ids['status_power']:
                        # 驱动电机状态帧 → 标记在线 + flags
                        if cur_status_time[u] == 0:
                            cur_status_time[u] = now
                        flags = data[7]
                        last_flags[u]['power'] = flags
                        # 堵转计时（驱动电机很少堵转，但保护加上）
                        dt = now - prev_rx_time[u]
                        if flags & 0x01:
                            stall_accum[u][1] += dt
                        else:
                            stall_accum[u][1] = 0.0
                        break
            except Exception:
                if running:
                    time.sleep(0.01)

    rx = threading.Thread(target=rx_loop, daemon=True)
    rx.start()

    # ── 启动串口遥控线程 ──
    if SERIAL_ENABLED:
        serial_thread = threading.Thread(target=serial_reader, daemon=True)
        serial_thread.start()
        print(f"[遥控] 串口监听线程已启动: {SERIAL_PORT} @ {SERIAL_BAUD}")
        print("  输入 'remote' 切换遥控模式")

    # 等待 CAN 状态帧
    print("等待 CAN 状态帧...")
    for _ in range(30):  # 最多等 3 秒
        alive = sum(1 for u in range(4) if cur_status_time[u] > 0)
        if alive >= 4:
            break
        time.sleep(0.1)
    print(f"  在线单元: {alive}/4")
    for u in range(4):
        if cur_status_time[u] > 0:
            print(f"  UNIT{u+1}: ticks={cur_raw_ticks[u]:5d}  abs={abs_ticks[u]:8d}  (0x{UNIT_CAN_IDS[u]['status_turn']:03X})")
    if alive < 4:
        print("  ⚠ 部分单元离线，请检查 CAN 总线连接")

    t = threading.Thread(target=cli_thread, daemon=True)
    t.start()

    print("=" * 60)
    print("  R1 舵轮底盘控制   vx vy omega → 4 单元协同")
    print("=" * 60)
    print("  命令: 0.5 0 0.1   → 前进+右转")
    print("        stop        → 停止")
    print("        s           → 显示状态")
    print("        cal         → 全部归零")
    print("        q           → 退出")
    print("        reset       → 重置堵转保护")
    print("        remote      → 切换遥控模式（串口 → 底盘）")
    print("        disable 4   → 禁用 UNIT4（坏单元排除）")
    print("        enable 4    → 重新启用 UNIT4")
    print("  ⚠ 必须先用 cal 归零，否则角度不准")
    print("-" * 60)

    cycle = 0
    try:
        while running:
            now = time.time()

            # ── 遥控命令更新 ──
            apply_remote_command()

            # ── 执行遥控指令（cal/stop/steer 等）──
            cmd_to_exec = None
            with remote_cmd_lock:
                if remote_cmd is not None:
                    cmd_to_exec = remote_cmd
                    remote_cmd = None
            if cmd_to_exec:
                parts = cmd_to_exec.split()
                cmd = parts[0]
                if cmd == 'cal':
                    # 类似 CLI 的 cal 逻辑
                    offline = [u+1 for u in range(4) if u not in DISABLED_UNITS and (time.time() - cur_status_time[u]) > 2.0]
                    if offline:
                        print(f"[遥控] ❌ UNIT{offline} 离线，无法 cal")
                    else:
                        for u in range(4):
                            if u not in DISABLED_UNITS:
                                set_speed(u, 'turn', 0)
                                set_speed(u, 'power', 0)
                        time.sleep(0.2)
                        for u in range(4):
                            ZERO_OFFSETS[u] = abs_ticks[u]
                            target_steer_deg[u] = 0.0
                            last_sent_steer[u] = 999
                            last_sent_drive[u] = 999
                        angle_mode = True
                        reset_protection()
                        print("[遥控] ✅ 远程 cal 完成，全部归零")
                elif cmd == 'stop':
                    for u in range(4):
                        if u not in DISABLED_UNITS:
                            set_speed(u, 'turn', 0)
                            set_speed(u, 'power', 0)
                    target_drive_speed = [0, 0, 0, 0]
                    last_sent_steer = [999, 999, 999, 999]
                    last_sent_drive = [999, 999, 999, 999]
                    angle_mode = False
                    reset_protection()
                    print("[遥控] ⏹ 远程 stop")
                elif cmd == 's':
                    print_status()
                elif cmd == 'reset':
                    reset_protection()
                    print("[遥控] 🔄 保护已重置")
                elif cmd == 'remote':
                    REMOTE_MODE = not REMOTE_MODE
                    print(f"[遥控] {'📡 遥控模式' if REMOTE_MODE else '⌨️ 键盘模式'}")
                elif cmd == 'disable' and len(parts) > 1:
                    try:
                        u = int(parts[1]) - 1
                        if 0 <= u < 4:
                            DISABLED_UNITS.add(u)
                            set_speed(u, 'turn', 0)
                            set_speed(u, 'power', 0)
                            print(f"[遥控] ❌ UNIT{u+1} 已禁用")
                    except: pass
                elif cmd == 'enable' and len(parts) > 1:
                    try:
                        u = int(parts[1]) - 1
                        DISABLED_UNITS.discard(u)
                        print(f"[遥控] ✅ UNIT{u+1} 已启用")
                    except: pass
                elif cmd == 'steer' and len(parts) > 1:
                    try:
                        sa = float(parts[1]) % 360
                        for u in range(4):
                            target_steer_deg[u] = (-sa) % 360.0
                            target_drive_speed[u] = 0
                        angle_mode = True
                        print(f"[遥控] 🎯 全部转向 {sa:.0f}°")
                    except: pass

            # ── 遥控超时保护 ──
            if REMOTE_MODE and (now - last_remote_time) > REMOTE_TIMEOUT and last_remote_time > 0:
                # 超过 1 秒没收到遥控数据 → 全车停止
                if any(s != 0 for s in last_sent_drive) or any(s != 0 for s in last_sent_steer):
                    can_send(0x101, bytes([CMD_STOP, 0, 0, 0, 0, 0, 0, 0]))
                    for u in range(4):
                        if u not in DISABLED_UNITS:
                            set_speed(u, 'turn', 0)
                            set_speed(u, 'power', 0)
                        last_sent_steer[u] = 999
                        last_sent_drive[u] = 999
                    target_drive_speed = [0, 0, 0, 0]
                    angle_mode = False
                    drive_ready = False
                    print("[遥控] ⚠️ 遥控超时，已紧急停止")
                    last_remote_time = 0  # 防止重复触发

            # ── 堵转保护检测（优先于控制命令）──
            for u in range(4):
                if u in DISABLED_UNITS:
                    continue
                if PROTECTION_ACTIVE[u]:
                    continue
                if stall_accum[u][0] > STALL_TIMEOUT:
                    PROTECTION_ACTIVE[u] = True
                    set_speed(u, 'turn', 0)
                    target_steer_deg[u] = angle_now(u)
                    if not stall_warned[u]:
                        print(f"⚠️ PROTECT: UNIT{u+1} 转向堵转 {STALL_TIMEOUT:.0f}s，已切断")
                        print(f"   输入 'reset' 解除保护（确认故障排除后）")
                        stall_warned[u] = True
                if stall_accum[u][1] > STALL_TIMEOUT:
                    PROTECTION_ACTIVE[u] = True
                    set_speed(u, 'power', 0)
                    target_drive_speed[u] = 0
                    if not stall_warned[u]:
                        print(f"⚠️ PROTECT: UNIT{u+1} 驱动堵转 {STALL_TIMEOUT:.0f}s，已切断")
                        stall_warned[u] = True

            # ── 转向控制 ──
            for u in range(4):
                if u in DISABLED_UNITS:
                    continue
                if not angle_mode:
                    # 非角度模式：重置积分
                    integral[u] = 0.0
                    prev_err[u] = 0.0
                    continue
                if PROTECTION_ACTIVE[u]:
                    continue
                angle = angle_now(u)
                err = shortest_error(angle, target_steer_deg[u])

                if abs(err) < DEAD_ZONE_DEG:
                    # 死区内：停发命令，冻结积分（不清零，保持用于克服静差）
                    send = 0
                else:
                    # PI 控制
                    clamped_err = max(-90.0, min(90.0, err))

                    # P 项
                    p_term = clamped_err * KP_ANGLE

                    # I 项（积分累计 + 限幅防 windup）
                    integral[u] += clamped_err * KI_ANGLE
                    integral[u] = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT, integral[u]))
                    # 误差反号时快速释放积分
                    if clamped_err * prev_err[u] < 0:
                        integral[u] *= 0.85

                    speed = int(p_term + integral[u])
                    speed = max(-MAX_STEER_SPEED, min(MAX_STEER_SPEED, speed))
                    # 最低速度 2（比原来降低 3→2，减少过冲）
                    if 0 < speed < 2:
                        speed = 2
                    elif -2 < speed < 0:
                        speed = -2
                    send = speed
                    prev_err[u] = clamped_err

                # 速率限制
                prev = last_sent_steer[u]
                if prev != 999 and abs(send - prev) > STEER_RAMP_RATE:
                    if send > prev:
                        send = prev + STEER_RAMP_RATE
                    else:
                        send = prev - STEER_RAMP_RATE

                if send != last_sent_steer[u]:
                    set_speed(u, 'turn', send)
                    last_sent_steer[u] = send

            # ── 驱动控制（等待舵向就绪再输出）──
            drive_ready_local = True
            if angle_mode:
                for u in range(4):
                    if u in DISABLED_UNITS or PROTECTION_ACTIVE[u]:
                        continue
                    err = shortest_error(angle_now(u), target_steer_deg[u])
                    if abs(err) > STEER_READY_DEG:
                        drive_ready_local = False
                        break
            drive_ready = drive_ready_local

            if angle_mode and drive_ready:
                for u in range(4):
                    if u in DISABLED_UNITS or PROTECTION_ACTIVE[u]:
                        continue
                    send = target_drive_speed[u]
                    if send != last_sent_drive[u]:
                        set_speed(u, 'power', send)
                        last_sent_drive[u] = send
            elif angle_mode and not drive_ready:
                # 舵向未就绪：驱动必须为 0
                for u in range(4):
                    if u in DISABLED_UNITS:
                        continue
                    if last_sent_drive[u] != 0:
                        set_speed(u, 'power', 0)
                        last_sent_drive[u] = 0

            cycle += 1
            if cycle % 10 == 0:
                print_status()
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n全车停止...")
        for u in range(4):
            set_speed(u, 'turn', 0)
            set_speed(u, 'power', 0)
        can_bus.shutdown()


if __name__ == '__main__':
    main()
