#!/usr/bin/env python3
"""
RS00 电机实时监控（纯被动监听）
只在 CAN 总线上监听，绝不发送任何指令，不干扰外部控制。

- --auto-report: 尝试开启 Type 24 主动上报查看（需固件 ≥0.0.3.0）

用法:
    python3 monitor.py                        # 纯被动监听
    python3 monitor.py --auto-report          # 尝试主动上报（需新固件）
"""

import sys
import time
import socket
import struct
from rs00_control import (CAN_EFF_MASK, parse_feedback,
                          _create_can_socket,
                          _send_raw,
                          float_to_uint, build_ext_id,
                          P_MIN, P_MAX, V_MIN, V_MAX,
                          KP_MIN, KP_MAX, KD_MIN, KD_MAX,
                          T_MIN, T_MAX)


class MotorCache:
    """每个电机的缓存状态"""
    def __init__(self, mid):
        self.mid = mid
        self.state = None
        self.last_type2 = 0

        # 从总线捕获的最近 Type 1 参数（用于心跳重放）
        self.last_pos = 0.0
        self.last_vel = 0.0
        self.last_kp = 0.0
        self.last_kd = 0.0
        self.have_cmd = False


def parse_type1(data):
    """解析 Type 1 数据 → (pos, vel, kp, kd) 物理值"""
    if not data or len(data) < 8:
        return None
    def u2f(raw, mn, mx):
        return raw * (mx - mn) / 65535 + mn
    return {
        "pos": u2f((data[0]<<8)|data[1], P_MIN, P_MAX),
        "vel": u2f((data[2]<<8)|data[3], V_MIN, V_MAX),
        "kp":  u2f((data[4]<<8)|data[5], KP_MIN, KP_MAX),
        "kd":  u2f((data[6]<<8)|data[7], KD_MIN, KD_MAX),
    }


def _name(mid):
    return {1: "肩膀#1", 2: "肘部#2"}.get(mid, f"电机#{mid}")


def send_heartbeat(sock, mid, mc):
    """发心跳查询（重放最后 Type 1 参数 or 零指令）"""
    if mc.have_cmd:
        pos, vel, kp, kd = mc.last_pos, mc.last_vel, mc.last_kp, mc.last_kd
    else:
        pos = vel = kp = kd = 0.0
    p = float_to_uint(pos, P_MIN, P_MAX)
    v = float_to_uint(vel, V_MIN, V_MAX)
    kp_u = float_to_uint(kp, KP_MIN, KP_MAX)
    kd_u = float_to_uint(kd, KD_MIN, KD_MAX)
    tq_u = float_to_uint(0, T_MIN, T_MAX)
    data = [(p>>8)&0xFF, p&0xFF, (v>>8)&0xFF, v&0xFF,
            (kp_u>>8)&0xFF, kp_u&0xFF, (kd_u>>8)&0xFF, kd_u&0xFF]
    _send_raw(sock, build_ext_id(1, tq_u, mid), data)


def main():
    import argparse
    parser = argparse.ArgumentParser(description="RS00 电机实时监控")
    parser.add_argument("--iface", default="can0", help="CAN 接口")
    parser.add_argument("--id", type=int, default=0, choices=[0, 1, 2],
                        help="监听哪个: 0=两个(默认)")
    parser.add_argument("--auto-report", action="store_true",
                        help="尝试 Type 24 自动上报（需固件 ≥0.0.3.0）")
    parser.add_argument("--interval", type=int, default=10,
                        help="上报间隔ms (默认10, 需 --auto-report)")
    args = parser.parse_args()

    watch_ids = [1, 2] if args.id == 0 else [args.id]
    caches = {mid: MotorCache(mid) for mid in watch_ids}

    sock = _create_can_socket(args.iface, timeout=0.3)
    if sock is None:
        print(f"\n⚠️  CAN 接口 {args.iface} 不可用，请先 CanCmd")
        sys.exit(1)

    auto_ok = False
    if args.auto_report:
        print("\n  尝试开启主动上报 (Type 24)...")
        from rs00_control import enable_auto_report
        try:
            for mid in watch_ids:
                enable_auto_report(args.iface, mid, interval_ms=args.interval)
            auto_ok = True
            print("  ✅ 开启成功")
        except Exception as e:
            print(f"  ⚠️ 开启失败 ({e})，退回心跳模式")
        time.sleep(0.15)

        # 验证是否真的有持续帧
        test_sock = _create_can_socket(args.iface, timeout=0.3)
        frames = 0
        deadline = time.time() + 0.5
        while time.time() < deadline:
            try:
                test_sock.recv(16)
                frames += 1
            except socket.timeout:
                break
        test_sock.close()
        if frames < 5:
            print(f"  ⚠️ 只收到 {frames} 帧，自动上报未生效（固件可能过旧）")
            auto_ok = False
            from rs00_control import disable_auto_report
            for mid in watch_ids:
                disable_auto_report(args.iface, mid)
        else:
            print(f"  ✅ 确认持续帧流 ({frames}帧/0.5s = ~{frames*2}Hz)")

    mode_name = "主动上报" if auto_ok else "被动监听"
    print()
    print("=" * 75)
    print(f"  RS00 电机实时监控  —  {mode_name}模式")
    print("  纯被动监听，不会发送任何指令，不干扰外部控制")
    print("  可在其他终端同时用 cansend / Python REPL 控制")
    if auto_ok:
        print("  电机已开启自动上报，数据持续刷新")
    print("  Ctrl+C 退出")
    print("=" * 75)

    hdr = []
    for mid in watch_ids:
        hdr.append(f"{_name(mid):>10}: {'角度(°)':>8} {'速度(°/s)':>7} "
                   f"{'力矩(Nm)':>7} {'温度(°C)':>5} {'模式':>5}{'延时':>6}")
    print("  " + "  ".join(hdr))

    last_display = 0
    total_frames = 0
    last_heartbeat = {mid: 0 for mid in watch_ids}

    try:
        while True:
            now = time.time()

            # ── 接收 ──
            try:
                frame = sock.recv(16)
                total_frames += 1
                can_id_raw = struct.unpack('I', frame[:4])[0]
                ext_id = can_id_raw & CAN_EFF_MASK
                length = min(frame[4], 8)
                data = list(frame[8:8+length])
                fb_type = (ext_id >> 24) & 0x1F
                mid = (ext_id >> 8) & 0xFF
                if mid not in watch_ids:
                    continue

                if fb_type == 1:
                    p = parse_type1(data)
                    if p:
                        caches[mid].last_pos = p["pos"]
                        caches[mid].last_vel = p["vel"]
                        caches[mid].last_kp = p["kp"]
                        caches[mid].last_kd = p["kd"]
                        caches[mid].have_cmd = True
                elif fb_type in (2, 0x18):
                    s = parse_feedback(data)
                    if s:
                        caches[mid].state = s
                        caches[mid].last_type2 = now
            except socket.timeout:
                pass

            # ── 心跳已禁用：防止干扰外部控制 ──
            # 如需主动刷新数据，请用 --auto-report（需固件支持）
            # 原心跳逻辑会发送 Type 1 指令覆盖外部控制参数，已移除

            # ── 显示（~10Hz） ──
            if now - last_display < 0.1:
                continue
            last_display = now

            parts = []
            for mid in watch_ids:
                mc = caches[mid]
                s = mc.state
                age = now - mc.last_type2 if mc.last_type2 > 0 else 999
                if s and age < 6:
                    tag = "上报" if auto_ok else ("心跳" if age > 1.5 else "实时")
                    # 显示距上次更新的毫秒数
                    ms = age * 1000
                    if ms >= 1000:
                        time_str = f"{age:.1f}s"
                    else:
                        time_str = f"{ms:.0f}ms"
                    parts.append(
                        f"{_name(mid):>10}: {s['position']:>7.1f}°  "
                        f"{s['velocity']:>+6.1f}°/s "
                        f"{s['torque']:>+6.3f}Nm "
                        f"{s['temperature']:>5.1f}°C "
                        f"{tag:>5}{time_str:>6}"
                    )
                else:
                    parts.append(f"{_name(mid):>10}: {'---':>8} {'---':>8} "
                                 f"{'---':>8} {'---':>6} {'---':>7}")
            sys.stdout.write("\r  " + "  ".join(parts))
            sys.stdout.flush()

    except KeyboardInterrupt:
        sock.close()
        if auto_ok:
            print("\n\n  关闭主动上报...")
            from rs00_control import disable_auto_report
            for mid in watch_ids:
                disable_auto_report(args.iface, mid)
        print(f"\n✅ 监控结束 ({total_frames} 帧)")
        sys.exit(0)


if __name__ == "__main__":
    main()
