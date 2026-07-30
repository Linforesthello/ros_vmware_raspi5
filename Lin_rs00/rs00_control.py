#!/usr/bin/env python3
"""
RS00 电机 Linux (slcan) 控制脚本 — 私有协议 (CAN 扩展帧 29-bit ID)

速查:
  - 电机默认 CAN ID = 127 (0x7F)
  - 主机 ID = 0xFD (253)
  - 波特率 = 1 Mbps
  - 参考: RS00_速查卡.md, RS00_新手教程.md

硬件: CANable2 → /dev/ttyACM1 → slcan → can0
"""

import os
import glob
import subprocess
import time
import argparse
import socket
import struct

# ─── 常量 (角度制; 转换: rad × 180/π = °) ───
P_MIN, P_MAX = -720.0, 720.0   # 位置范围 (°)  对应 -4π ~ +4π rad
V_MIN, V_MAX = -1891.0, 1891.0 # 速度范围 (°/s) 对应 ±33 rad/s
KP_MIN, KP_MAX = 0.0, 500.0    # Kp 范围
KD_MIN, KD_MAX = 0.0, 5.0      # Kd 范围
T_MIN, T_MAX = -14.0, 14.0     # 力矩范围 (N.m) — 前馈用


def float_to_uint(x, x_min, x_max, bits=16):
    """将浮点数 (角度/速度等) 映射到 uint 范围"""
    span = x_max - x_min
    if x > x_max:
        x = x_max
    elif x < x_min:
        x = x_min
    return int((x - x_min) * ((1 << bits) - 1) / span)


def build_ext_id(mode_type, data_field, motor_id=127):
    """
    构建 29-bit 扩展帧 ID
    bit28~24: mode_type (通信类型)
    bit23~8 : data_field (Type 1=力矩, Type 3/4=主机ID)
    bit7~0  : motor_id (目标电机ID)
    """
    return ((mode_type & 0x1F) << 24) | ((data_field & 0xFFFF) << 8) | (motor_id & 0xFF)


def cansend(iface, ext_id, data=None):
    """发送 CAN 扩展帧，返回 True/False 表示发送是否成功"""
    if data is None:
        data = [0] * 8
    ext_hex = f"{ext_id:08X}"
    dat_hex = "".join(f"{b:02X}" for b in data)
    cmd = f"cansend {iface} {ext_hex}#{dat_hex}"
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=1)
    if result.returncode != 0:
        print(f"  [ERR] {result.stderr.strip()}")
    return result.returncode == 0


def motor_enable(iface, motor_id=127, master_id=0xFD):
    """Type 3: 使能电机 (data=主机ID)"""
    cansend(iface, build_ext_id(3, master_id, motor_id))
    print(f"  [Type 3] 使能电机 ID={motor_id}")


def motor_disable(iface, motor_id=127, master_id=0xFD):
    """Type 4: 停止电机 (data=主机ID)"""
    cansend(iface, build_ext_id(4, master_id, motor_id))
    print(f"  [Type 4] 停止电机 ID={motor_id}")


def motor_control(iface, motor_id=127, master_id=0xFD,
                  pos=0.0, vel=0.0, kp=0.0, kd=0.0, torque=0.0):
    """
    Type 1: 操作控制模式指令
    公式: t_ref = Kd×(v_set − v_actual) + Kp×(p_set − p_actual) + t_ff
    CAN ID bit23~8 = 前馈力矩 (t_ff)
    数据区 8 字节: position(2B) + velocity(2B) + Kp(2B) + Kd(2B)
    单位: pos=°, vel=°/s, torque=N.m
    """
    p = float_to_uint(pos, P_MIN, P_MAX)
    v = float_to_uint(vel, V_MIN, V_MAX)
    kp_u = float_to_uint(kp, KP_MIN, KP_MAX)
    kd_u = float_to_uint(kd, KD_MIN, KD_MAX)
    tq = float_to_uint(torque, T_MIN, T_MAX)

    data = [
        (p >> 8) & 0xFF, p & 0xFF,
        (v >> 8) & 0xFF, v & 0xFF,
        (kp_u >> 8) & 0xFF, kp_u & 0xFF,
        (kd_u >> 8) & 0xFF, kd_u & 0xFF,
    ]
    cansend(iface, build_ext_id(1, tq, motor_id), data)
    print(f"  [Type 1] pos={pos:.1f}°  vel={vel:.1f}°/s  kp={kp:.1f}  kd={kd:.1f}  torque={torque:.2f}Nm")


def set_mode(iface, motor_id=127, master_id=0xFD, mode=0):
    """
    Type 18: 设置运行模式 (掉电丢失)
    mode: 0=运控, 1=PP位置, 2=速度, 3=电流, 5=CSP位置
    参数索引: 0x7005 = run_mode
    """
    mode_names = {0: "运控", 1: "PP位置", 2: "速度", 3: "电流", 5: "CSP位置"}
    write_param(iface, 0x7005, [mode & 0xFF, 0, 0, 0], motor_id, master_id)
    print(f"  [Type 18] 设置模式 → {mode_names.get(mode, str(mode))}")


def read_param(iface, motor_id=127, master_id=0xFD, index=0x3022):
    """
    Type 17: 读取单个参数
    注: 仅发送读请求，需配合 candump 查看应答帧的 Byte4~7
    常用: 0x3022=故障码, 0x700B=力矩限制, 0x7019=状态字
    返回: bool (发送成功/失败)
    """
    ext_id = build_ext_id(0x11, master_id, motor_id)
    data = [
        index & 0xFF, (index >> 8) & 0xFF,
        0x00, 0x00,
        0x00, 0x00, 0x00, 0x00
    ]
    return cansend(iface, ext_id, data)


def write_param(iface, index, value_bytes, motor_id=127, master_id=0xFD):
    """
    Type 18: 写参数 (参考手册第 42 页)
    帧结构: 扩展帧ID = 0x12<<24 | 主机<<8 | 电机ID
    Byte0~1: 参数索引 (小端序)
    Byte2~3: 0x00 0x00
    Byte4~7: 参数值 (小端序)
    """
    data = [index & 0xFF, (index >> 8) & 0xFF, 0, 0] + list(value_bytes)
    while len(data) < 8:
        data.append(0)
    cansend(iface, build_ext_id(0x12, master_id, motor_id), data)


def set_can_id(iface, new_id, master_id=0xFD, current_id=127):
    """
    Type 7: 设置电机 CAN ID (立即生效)
    扩展帧 ID = (0x7<<24) | (新ID<<16) | (主机ID<<8) | 当前电机ID
    ⚠️ 一次只接一个电机到总线上操作!
    """
    data_field = (new_id << 8) | master_id
    cansend(iface, build_ext_id(7, data_field, current_id))
    print(f"  [Type 7] 电机 CAN ID: {current_id} → {new_id}")
    print(f"  ⚠️  验证: cansend {iface} 0300FD{new_id:02X}#")


def set_zero_motor(iface, motor_id=127, master_id=0xFD):
    """
    Type 6: 设置电机机械零位。

    将当前位置设为 0°。Byte0 必须为 0x01。
    响应: Type 2 应答帧。
    """
    cansend(iface, build_ext_id(6, master_id, motor_id),
            [0x01, 0, 0, 0, 0, 0, 0, 0])
    print(f"  [Type 6] 电机 ID={motor_id} 机械零位已设置")


def enable_auto_report(iface, motor_id=127, master_id=0xFD, interval_ms=10):
    """
    Type 24: 开启电机主动上报。

    开启后电机以固定间隔持续发送 Type 2 反馈帧 (角度/速度/力矩/温度)，
    无需逐条指令触发。

    参数:
        interval_ms: 上报间隔 (毫秒, 最小10, 步进5, 默认10)
                     EPScan_time = 1 + (interval_ms - 10) / 5
    """
    # 设置上报间隔 (0x7026)
    eps_time = max(1, int(1 + (interval_ms - 10) / 5))
    write_param(iface, 0x7026, struct.pack('<H', eps_time), motor_id, master_id)
    time.sleep(0.05)

    # Type 24: 开启上报 (F_CMD=0x01)
    cansend(iface, build_ext_id(0x18, master_id, motor_id),
            [0, 0, 0, 0, 0, 0, 0, 0x01])
    print(f"  [Type 24] 电机 ID={motor_id} 主动上报已开启 (间隔{interval_ms}ms)")


def disable_auto_report(iface, motor_id=127, master_id=0xFD):
    """Type 24: 关闭电机主动上报。"""
    cansend(iface, build_ext_id(0x18, master_id, motor_id),
            [0, 0, 0, 0, 0, 0, 0, 0x00])
    print(f"  [Type 24] 电机 ID={motor_id} 主动上报已关闭")


# ─── CAN 帧接收 (Python socket CAN) ───

CAN_EFF_FLAG = 0x80000000  # 扩展帧标志
CAN_EFF_MASK = 0x1FFFFFFF   # 29-bit ID 掩码


def _create_can_socket(iface, timeout=0.1):
    """创建 CAN RAW socket (需 root 或 cap_net_raw)"""
    try:
        sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind((iface,))
        sock.settimeout(timeout)
        return sock
    except OSError as e:
        print(f"  [ERR] 无法创建 CAN socket: {e}")
        print(f"  [HINT] 需 sudo 或将用户加入 netdev 组")
        return None


def can_recv(iface="can0", timeout=0.1):
    """
    接收一个 CAN 扩展帧。

    返回 (ext_id_29bit, data_bytes_list) 或 None (超时/失败)。

    示例:
        result = can_recv("can0", timeout=0.5)
        if result:
            ext_id, data = result
            fb_type = (ext_id >> 24) & 0x1F  # 通信类型
    """
    sock = _create_can_socket(iface, timeout)
    if sock is None:
        return None

    try:
        frame = sock.recv(16)  # can_frame 固定 16 字节
        sock.close()
    except socket.timeout:
        sock.close()
        return None
    except OSError as e:
        print(f"  [ERR] CAN recv: {e}")
        sock.close()
        return None

    # 解析 can_frame: can_id(4B) + len(1B) + pad(1B) + res0(1B) + res1(1B) + data(8B)
    can_id_raw = struct.unpack('I', frame[:4])[0]
    length = min(frame[4], 8)
    data = list(frame[8:8 + length])

    # 提取 29-bit 扩展帧 ID
    ext_id = can_id_raw & CAN_EFF_MASK if (can_id_raw & CAN_EFF_FLAG) else None
    return (ext_id, data)


def parse_feedback(data):
    """
    解析 Type 2 应答帧数据 (8 字节)。

    返回 dict:
        position(°)  velocity(°/s)  torque(N.m)  temperature(°C)
    或 None (数据不足)
    """
    if not data or len(data) < 8:
        return None

    p_raw = (data[0] << 8) | data[1]
    v_raw = (data[2] << 8) | data[3]
    t_raw = (data[4] << 8) | data[5]
    temp_raw = (data[6] << 8) | data[7]

    return {
        "position":    round(p_raw * (P_MAX - P_MIN) / 65535 + P_MIN, 1),
        "velocity":    round(v_raw * (V_MAX - V_MIN) / 65535 + V_MIN, 1),
        "torque":      round(t_raw * (T_MAX - T_MIN) / 65535 + T_MIN, 3),
        "temperature": round(temp_raw / 10.0, 1),
    }


def _send_raw(sock, ext_id, data):
    """
    通过已有 socket 发送 CAN 扩展帧（替代 subprocess cansend）。
    data: list of int (0-255)
    """
    can_id = ext_id | CAN_EFF_FLAG
    frame = struct.pack('I', can_id) + bytes([len(data), 0, 0, 0]) + bytes(data)
    sock.send(frame)


def _recv_raw(sock):
    """
    从已有 socket 读取一个 CAN 帧。
    返回 (ext_id, data_list) 或 None (超时)。
    """
    try:
        frame = sock.recv(16)
    except socket.timeout:
        return None
    can_id_raw = struct.unpack('I', frame[:4])[0]
    ext_id = can_id_raw & CAN_EFF_MASK
    length = min(frame[4], 8)
    data = list(frame[8:8 + length])
    return (ext_id, data)


def get_motor_state(iface, motor_id=127, master_id=0xFD, timeout=0.15,
                    kp_hold=0, kd_hold=0, pos_hold=0, vel_hold=0):
    """
    查询电机当前状态。

    用同一 socket 收发，消除子进程 cansend 的竞争窗口。
    返回 dict 或 None (超时/无应答)。

    参数:
        kp_hold:  查询时保持的 Kp 值
        kd_hold:  查询时保持的 Kd 值
        pos_hold: 查询时保持的目标位置（设为当前控制目标，避免电机被拽离）
        vel_hold: 查询时保持的目标速度
    """
    # 保持当前控制参数，只触发应答不改变电机行为
    p = float_to_uint(pos_hold, P_MIN, P_MAX)
    v = float_to_uint(vel_hold, V_MIN, V_MAX)
    kp = float_to_uint(kp_hold, KP_MIN, KP_MAX)
    kd = float_to_uint(kd_hold, KD_MIN, KD_MAX)
    tq = float_to_uint(0, T_MIN, T_MAX)
    data = [
        (p >> 8) & 0xFF, p & 0xFF,
        (v >> 8) & 0xFF, v & 0xFF,
        (kp >> 8) & 0xFF, kp & 0xFF,
        (kd >> 8) & 0xFF, kd & 0xFF,
    ]
    ext_id = build_ext_id(1, tq, motor_id)

    # 同一 socket 收发，验证应答帧来自目标电机
    sock = _create_can_socket(iface, timeout)
    if sock is None:
        return None

    _send_raw(sock, ext_id, data)

    deadline = time.time() + timeout
    state = None
    while True:
        remaining = deadline - time.time()
        if remaining <= 0:
            break
        sock.settimeout(min(remaining, 0.05))
        result = _recv_raw(sock)
        if result is None:
            continue  # 超时未到，继续等
        rx_id, rx_data = result

        # 验证应答来自目标电机 (Type 2: bit15~8 = 回复电机ID)
        resp_motor = (rx_id >> 8) & 0xFF
        if resp_motor != motor_id:
            continue  # 不是目标电机的帧，跳过

        fb_type = (rx_id >> 24) & 0x1F
        if fb_type != 2:
            continue

        state = parse_feedback(rx_data)
        if state:
            state["motor_id"] = motor_id
        break

    sock.close()
    return state


def find_can_devices():
    """自动扫描可用的 CANable / 串口设备"""
    devices = []
    for pattern in ["/dev/ttyACM*", "/dev/ttyUSB*"]:
        devices.extend(glob.glob(pattern))
    return sorted(devices)


def select_device(default="/dev/ttyACM1"):
    """
    检测可用设备，若默认设备不存在则让用户手动选择。
    返回选中的设备路径。
    """
    if os.path.exists(default):
        return default

    print(f"\n⚠️  默认设备 {default} 不存在，正在扫描可用设备...")
    devices = find_can_devices()

    if not devices:
        print("\n❌ 未检测到任何串口设备。请确认：")
        print("   1. CANable (或类似设备) 已插入 USB 口")
        print("   2. 虚拟机已把设备直通给 Linux (VMware: 可移动设备 → 连接)")
        print("   3. 驱动已安装 (ls /dev/ttyACM* /dev/ttyUSB*)")
        print("   4. 或手动指定: python3 rs00_control.py --device /dev/ttyACM0\n")
        return None

    print("\n🔍 检测到以下串口设备：")
    for i, dev in enumerate(devices, 1):
        print(f"   {i}. {dev}")

    while True:
        try:
            raw = input(f"\n请选择 (1-{len(devices)}) 或输入完整路径 [1]: ").strip()
            if not raw:
                raw = "1"
            # 数字选择
            idx = int(raw) - 1
            if 0 <= idx < len(devices):
                return devices[idx]
            print(f"   请输入 1-{len(devices)}")
        except ValueError:
            # 当成路径处理
            raw = raw.strip()
            if os.path.exists(raw):
                return raw
            print(f"   路径 '{raw}' 不存在，请重试")


def setup_can(device="/dev/ttyACM1", baud="s8", interface="can0"):
    """配置 slcan CAN 接口 (波特率对照: s4=125k, s5=250k, s6=500k, s8=1M)"""
    print(f"\n🔧 设置 CAN 接口 {interface} 在 {device} 上 (波特率: {baud})")

    # 清理旧接口
    subprocess.run(f"sudo ip link set {interface} down 2>/dev/null", shell=True)
    subprocess.run(f"sudo pkill -f 'slcand.*{interface}' 2>/dev/null", shell=True)
    time.sleep(0.5)

    # 启动 slcand
    subprocess.run(f"sudo slcand -o -c -{baud} {device} {interface}", shell=True, check=True)
    time.sleep(1)

    # 启动接口
    subprocess.run(f"sudo ip link set {interface} up", shell=True, check=True)
    print(f"  ✅ {interface} is UP")

    # 显示状态
    result = subprocess.run(f"ip link show {interface}", shell=True,
                            capture_output=True, text=True)
    print(f"  {result.stdout.strip()}")
    return interface


def main():
    parser = argparse.ArgumentParser(description="RS00 电机控制工具 (via slcan)")
    parser.add_argument("--device", default=None,
                        help="CANable 设备路径 (默认自动检测)")
    parser.add_argument("--iface", default="can0", help="CAN 接口名称")
    parser.add_argument("--baud", default="s8",
                        help="波特率: s6=500k, s8=1M (默认)")
    parser.add_argument("--id", type=int, default=127,
                        help="电机 CAN ID (默认 0x7F)")
    parser.add_argument("--master", type=int, default=0xFD,
                        help="主机 ID (默认 0xFD)")
    parser.add_argument("--no-setup", action="store_true",
                        help="不自动设置 slcan (接口已 UP)")
    args = parser.parse_args()

    iface = args.iface
    if not args.no_setup:
        device = args.device if args.device else select_device()
        if device is None:
            return
        setup_can(device, args.baud, iface)

    mid, mast = args.id, args.master
    print(f"\n🎯 目标电机 ID={mid}  主机 ID=0x{mast:02X}")
    print("=" * 50)

    # ─── 测试序列 (角度制, 参考 RS00_新手教程.md) ───
    print("\n1️⃣  使能 + 慢速旋转 (30°/s, Kd=1)...")
    motor_enable(iface, mid, mast)
    time.sleep(0.5)
    motor_control(iface, mid, mast, pos=0, vel=30, kp=0, kd=1)
    time.sleep(3)

    print("\n2️⃣  加速到 90°/s...")
    motor_control(iface, mid, mast, pos=0, vel=90, kp=0, kd=1)
    time.sleep(3)

    print("\n3️⃣  停止")
    motor_disable(iface, mid, mast)
    time.sleep(1)

    print("\n4️⃣  再使能, 位置控制 → 180° (Kp=1, Kd=1)")
    motor_enable(iface, mid, mast)
    time.sleep(0.5)
    motor_control(iface, mid, mast, pos=180.0, vel=0, kp=1, kd=1)
    time.sleep(3)

    print("\n5️⃣  回到 0°...")
    motor_control(iface, mid, mast, pos=0, vel=0, kp=1, kd=1)
    time.sleep(3)

    print("\n6️⃣  最终停止")
    motor_disable(iface, mid, mast)

    print("\n✅ 完成!")


if __name__ == "__main__":
    main()
