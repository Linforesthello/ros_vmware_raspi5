#!/usr/bin/env python3
"""
RS00 电机实时监控 — 混合模式

- 被动监听: 捕获 CAN 总线上的指令和应答，实时更新
- 智能心跳: 总线静默超过 2 秒，自动重发最后指令刷新数据
  （重发的是上次捕获的指令参数，不改变电机行为）
- 可在另一个终端同时用 cansend 控制

用法:
    python3 monitor.py                        # 两个电机
    python3 monitor.py --id 1                 # 只肩膀
"""

import sys
import time
import socket
import struct
from rs00_control import (CAN_EFF_MASK, parse_feedback,
                          _create_can_socket, _send_raw,
                          float_to_uint, build_ext_id,
                          P_MIN, P_MAX, V_MIN, V_MAX,
                          KP_MIN, KP_MAX, KD_MIN, KD_MAX,
                          T_MIN, T_MAX)


# 电机最后状态缓存
class MotorCache:
    """每个电机的缓存状态"""
    def __init__(self, mid):
        self.mid = mid
        self.state = None           # 最新 Type 2 解析结果
        self.last_type2_time = 0    # 上次收到 Type 2 的时间

        # 最后捕获的 Type 1 指令参数（用于心跳重放）
        self.last_pos = 0.0
        self.last_vel = 0.0
        self.last_kp = 0.0
        self.last_kd = 0.0
        self.last_torque = 0.0
        self.have_command = False   # True 之后才重放，否则发零指令


def parse_type1_data(data):
    """解析 Type 1 数据区 (8字节) → (pos, vel, kp, kd, torque) 物理值"""
    if not data or len(data) < 8:
        return None
    p_raw = (data[0] << 8) | data[1]
    v_raw = (data[2] << 8) | data[3]
    kp_raw = (data[4] << 8) | data[5]
    kd_raw = (data[6] << 8) | data[7]
    # 从 uint16 映射回物理量
    span_p = P_MAX - P_MIN
    span_v = V_MAX - V_MIN
    span_kp = KP_MAX - KP_MIN
    span_kd = KD_MAX - KD_MIN
    span_t = T_MAX - T_MIN
    return {
        "pos": p_raw * span_p / 65535 + P_MIN,
        "vel": v_raw * span_v / 65535 + V_MIN,
        "kp":  kp_raw * span_kp / 65535 + KP_MIN,
        "kd":  kd_raw * span_kd / 65535 + KD_MIN,
        "torque": p_raw * span_t / 65535 + T_MIN,  # 从 CAN ID bit23~8
    }


def build_safe_query(mc):
    """根据缓存构建安全查询指令（不改变电机行为）"""
    if mc.have_command:
        return (mc.last_pos, mc.last_vel, mc.last_kp, mc.last_kd, mc.last_torque)
    else:
        return (0.0, 0.0, 0.0, 0.0, 0.0)


def send_query(sock, mid, mc):
    """发送一次查询指令到指定电机"""
    pos, vel, kp, kd, tq = build_safe_query(mc)
    p = float_to_uint(pos, P_MIN, P_MAX)
    v = float_to_uint(vel, V_MIN, V_MAX)
    kp_u = float_to_uint(kp, KP_MIN, KP_MAX)
    kd_u = float_to_uint(kd, KD_MIN, KD_MAX)
    tq_u = float_to_uint(tq, T_MIN, T_MAX)
    data = [
        (p >> 8) & 0xFF, p & 0xFF,
        (v >> 8) & 0xFF, v & 0xFF,
        (kp_u >> 8) & 0xFF, kp_u & 0xFF,
        (kd_u >> 8) & 0xFF, kd_u & 0xFF,
    ]
    ext_id = build_ext_id(1, tq_u, mid)
    _send_raw(sock, ext_id, data)


def _name(mid):
    return {1: "肩膀#1", 2: "肘部#2"}.get(mid, f"电机#{mid}")


def main():
    import argparse
    parser = argparse.ArgumentParser(description="RS00 电机实时监控 (混合模式)")
    parser.add_argument("--iface", default="can0", help="CAN 接口")
    parser.add_argument("--id", type=int, default=0, choices=[0, 1, 2],
                        help="监听哪个: 0=两个(默认)")
    parser.add_argument("--heartbeat", type=float, default=2.0,
                        help="心跳间隔秒数 (0=禁用, 默认2s)")
    args = parser.parse_args()

    watch_ids = [1, 2] if args.id == 0 else [args.id]
    caches = {mid: MotorCache(mid) for mid in watch_ids}

    # 创建 socket
    sock = _create_can_socket(args.iface, timeout=0.3)
    if sock is None:
        print(f"\n⚠️  CAN 接口 {args.iface} 不可用")
        print("   请在另一个终端先: CanCmd")
        sys.exit(1)

    print()
    print("=" * 75)
    print("  RS00 电机实时监控  —  混合模式")
    if args.heartbeat > 0:
        print(f"  心跳 {args.heartbeat}s | 监听 + 自动刷新")
        if args.id == 0:
            print(f"  焦点: 肩膀#1 + 肘部#2")
        else:
            print(f"  焦点: {_name(args.id)}")
    else:
        print("  纯被动监听 (无心跳)")
    print("  Ctrl+C 退出")
    print("=" * 75)

    # 表头
    hdr_parts = []
    for mid in watch_ids:
        hdr_parts.append(
            f"{_name(mid):>10}: "
            f"{'角度(°)':>8} {'速度(°/s)':>7} "
            f"{'力矩(Nm)':>7} {'温度(°C)':>5} "
            f"{'更新':>6}"
        )
    print("  " + "  ".join(hdr_parts))

    frame_count = 0

    try:
        while True:
            now = time.time()

            # ── 被动接收 ──
            try:
                frame = sock.recv(16)
                frame_count += 1

                can_id_raw = struct.unpack('I', frame[:4])[0]
                ext_id = can_id_raw & CAN_EFF_MASK
                length = min(frame[4], 8)
                data = list(frame[8:8 + length])
                fb_type = (ext_id >> 24) & 0x1F
                motor_id = (ext_id >> 8) & 0xFF

                if motor_id not in watch_ids:
                    pass  # 忽略不关注的电机

                elif fb_type == 1:
                    # 捕获 Type 1 指令 → 记住参数
                    parsed = parse_type1_data(data)
                    if parsed and motor_id in caches:
                        mc = caches[motor_id]
                        mc.last_pos = parsed["pos"]
                        mc.last_vel = parsed["vel"]
                        mc.last_kp = parsed["kp"]
                        mc.last_kd = parsed["kd"]
                        mc.have_command = True

                elif fb_type == 2:
                    # Type 2 应答 → 更新状态
                    state = parse_feedback(data)
                    if state and motor_id in caches:
                        state["motor_id"] = motor_id
                        caches[motor_id].state = state
                        caches[motor_id].last_type2_time = now

            except socket.timeout:
                pass

            # ── 心跳: 静默超时后自动刷新 ──
            if args.heartbeat > 0:
                need_beat = False
                for mid, mc in caches.items():
                    age = now - mc.last_type2_time
                    if age > args.heartbeat:
                        need_beat = True
                        # 用 socket 直接发，复用已有连接
                        send_query(sock, mid, mc)
                        # 注意: 发完后下一轮循环 recv 会收到应答
                        caches[mid].last_type2_time = now - args.heartbeat * 0.8

                if need_beat:
                    # 发完心跳后等一下，让应答回来
                    time.sleep(0.05)

            # ── 显示 ──
            disp_parts = []
            for mid in watch_ids:
                mc = caches[mid]
                s = mc.state
                age = now - mc.last_type2_time if mc.last_type2_time > 0 else 999

                if s and (age < 10 or args.heartbeat > 0):
                    p = s["position"]
                    v = s["velocity"]
                    t = s["torque"]
                    temp = s["temperature"]

                    if age < 1.0:
                        status = f"{age:>5.1f}s●"
                    elif age < args.heartbeat * 1.5:
                        status = f"{age:>5.1f}s◐"
                    else:
                        status = f"{age:>5.1f}s○"

                    # 心跳标记
                    beat_flag = " ♥" if args.heartbeat > 0 and \
                        mc.have_command and age < args.heartbeat * 1.2 else ""

                    disp_parts.append(
                        f"{_name(mid):>10}: "
                        f"{p:>7.1f}°  "
                        f"{v:>+6.1f}°/s "
                        f"{t:>+6.3f}Nm "
                        f"{temp:>5.1f}°C "
                        f"{status}{beat_flag}"
                    )
                else:
                    disp_parts.append(
                        f"{_name(mid):>10}: "
                        f"{'等待':>7} {'指令中...':>8} "
                        f"{'':>8} {'':>6} {'':>9}"
                    )

            sys.stdout.write("\r  " + "  ".join(disp_parts))
            sys.stdout.flush()

    except KeyboardInterrupt:
        sock.close()
        print(f"\n\n✅ 监控结束 (共收到 {frame_count} 帧)")
        sys.exit(0)


if __name__ == "__main__":
    main()
