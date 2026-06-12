#!/usr/bin/env python3
"""
R2.py - 全向轮底盘键盘遥控程序

底盘: 正方形车体，四角各一个全向轮(90°辊子)，轮轴指向车体中心。
运动学: 键盘速度指令(vx, vy, ω) → 四轮逆运动学解算 → 串口电机指令。
"""

import serial
import curses
import time
import glob
import threading
import io
import math

# ==================== 配置 ====================

# 串口
BAUD_RATE = 115200

# 时序
MAIN_LOOP_TIMEOUT_MS = 10      # getch() 超时(ms)
RX_POLL_INTERVAL = 0.01        # RX 线程轮询间隔(s)
STATUS_LOG_INTERVAL = 0.3      # 日志刷新间隔(s)
DECAY = 0.85                   # 无按键时减速系数
KEY_TIMEOUT = 0.15             # 按键保持判定超时(s) — 覆盖终端键盘重复延迟

# 运动学参数
CHASSIS_HALF_DIAGONAL = 0.15   # 半对角线长 R(m) — 按实际车体尺寸改
WHEEL_SPEED_MAX = 35           # 单轮最大速度 (协议限 -35~35)
ROBOT_VEL_MAX = 30             # 最大平移速度
ROBOT_OMEGA_MAX = 100          # 最大旋转速度

# 电机 ID: 左前 右前 左后 右后
MOTOR_IDS = {
    'FL': 0x23, 'FR': 0x25,
    'RL': 0x24, 'RR': 0x26,
}

# 键盘 → (dx, dy, dω)
KEY_MAP = {
    ord('w'): ( 0,  1,  0),   # 前进
    ord('s'): ( 0, -1,  0),   # 后退
    ord('a'): (-1,  0,  0),   # 左移
    ord('d'): ( 1,  0,  0),   # 右移
    ord('q'): ( 0,  0,  1),   # 逆时针旋转
    ord('e'): ( 0,  0, -1),   # 顺时针旋转
}

# ==================== 串口 ====================

def select_port():
    ports = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
    if not ports:
        print("未找到可用串口，使用默认 /dev/ttyACM0")
        return '/dev/ttyACM0'
    print("可用串口：")
    for i, p in enumerate(ports):
        print(f"  {i}: {p}")
    while True:
        try:
            idx = int(input(f"请输入串口编号 (0-{len(ports)-1}): "))
            if 0 <= idx < len(ports):
                return ports[idx]
        except ValueError:
            pass
        print("输入无效，请重试")

SERIAL_PORT = select_port()
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
serial_lock = threading.Lock()
print(f"已连接: {SERIAL_PORT}")

# ==================== 逆运动学 ====================

INV_SQRT2 = 1.0 / math.sqrt(2)

def compute_wheel_speeds(vx, vy, omega):
    """
    全向轮逆运动学: 机器人速度(vx, vy, ω) → 四轮轮速

    正方形车体，四角全向轮(90°辊子)，轮轴指向中心。
    vx: 右为正 | vy: 前为正 | ω: 逆时针为正
    """
    R = CHASSIS_HALF_DIAGONAL
    s = {}
    s['FL'] = ( vx + vy) * INV_SQRT2 - R * omega
    s['FR'] = ( vx - vy) * INV_SQRT2 - R * omega
    s['RL'] = (-vx + vy) * INV_SQRT2 - R * omega
    s['RR'] = (-vx - vy) * INV_SQRT2 - R * omega

    # 归一化到最大轮速
    peak = max(abs(v) for v in s.values())
    if peak > WHEEL_SPEED_MAX:
        scale = WHEEL_SPEED_MAX / peak
        for k in s:
            s[k] = s[k] * scale

    return {k: int(round(v)) for k, v in s.items()}


def build_speed_cmd(motor_id, speed):
    speed = max(-WHEEL_SPEED_MAX, min(WHEEL_SPEED_MAX, speed))
    return f'AA 01 {motor_id:02X} 01 00 00 02 11 {speed & 0xFF:02X}'

# ==================== 串口收发 / 日志 ====================

def refresh_log(log_lines, log_win, lock):
    with lock:
        log_win.erase()
        h, w = log_win.getmaxyx()
        for i, line in enumerate(log_lines[-(h - 2):]):
            try:
                log_win.addstr(i, 0, line[:w - 1])
            except curses.error:
                pass
        log_win.refresh()


def send_command(cmd_str, log_lines, log_win, lock, do_log=True):
    ts = time.strftime('%H:%M:%S')
    try:
        with serial_lock:
            ser.write(bytes.fromhex(cmd_str.replace(' ', '')))
    except (ValueError, serial.SerialException) as e:
        if do_log:
            with lock:
                log_lines.append(f"[{ts}] ERR: {e}")
            refresh_log(log_lines, log_win, lock)
        return
    if do_log:
        with lock:
            log_lines.append(f"[{ts}] TX: {cmd_str.upper()}")
        refresh_log(log_lines, log_win, lock)


def send_velocity(speeds, log_lines, log_win, lock, do_log=True):
    """四轮速度逐条发送（锁内原子操作），只记一行日志"""
    ts = time.strftime('%H:%M:%S')
    # 预构建指令（不在锁内做字符串操作）
    cmds = []
    for name, mid in MOTOR_IDS.items():
        cmds.append(bytes.fromhex(build_speed_cmd(mid, speeds[name]).replace(' ', '')))
    try:
        with serial_lock:
            for b in cmds:
                ser.write(b)
    except Exception as e:
        if do_log:
            with lock:
                log_lines.append(f"[{ts}] ERR: {e}")
            refresh_log(log_lines, log_win, lock)
        return
    if do_log:
        desc = (f"FL={speeds['FL']:3d} FR={speeds['FR']:3d} "
                f"RL={speeds['RL']:3d} RR={speeds['RR']:3d}")
        with lock:
            log_lines.append(f"[{ts}] {desc}")
        refresh_log(log_lines, log_win, lock)


def rx_reader(log_lines, log_win, lock, stop_event):
    buf = io.BytesIO()
    while not stop_event.is_set():
        try:
            with serial_lock:
                raw = ser.read(ser.in_waiting) if ser.in_waiting > 0 else b''
            if not raw:
                time.sleep(RX_POLL_INTERVAL)
                continue
            buf.write(raw)
            buf.seek(0)
            lines = buf.read().split(b'\n')
            buf.seek(0)
            buf.truncate()
            buf.write(lines[-1])
            ts = time.strftime('%H:%M:%S')
            for line in lines[:-1]:
                text = line.decode('ascii', errors='replace').strip()
                if text:
                    with lock:
                        log_lines.append(f"[{ts}] RX: {text}")
                    refresh_log(log_lines, log_win, lock)
        except serial.SerialException:
            ts = time.strftime('%H:%M:%S')
            with lock:
                log_lines.append(f"[{ts}] ERR: 串口断线，RX 线程退出")
            refresh_log(log_lines, log_win, lock)
            break
        time.sleep(RX_POLL_INTERVAL)

# ==================== 主循环 ====================

def main(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(1)
    stdscr.timeout(MAIN_LOOP_TIMEOUT_MS)

    help_text = [
        "=== 全向轮底盘键盘遥控 | ESC 退出 ===",
        "  W/S: 前进/后退   A/D: 左移/右移",
        "  Q/E: 旋转(逆/顺时针)   空格: 急停",
        "  即时发送指令                                ",
        "-" * 50,
    ]

    rows, cols = stdscr.getmaxyx()
    for i, line in enumerate(help_text):
        try:
            stdscr.addstr(i, 0, line[:cols - 1])
        except curses.error:
            pass
    stdscr.refresh()

    log_rows = rows - len(help_text)
    log_win = curses.newwin(log_rows, cols, len(help_text), 0)
    log_lines = []
    lock = threading.Lock()

    vx = vy = omega = 0.0

    # 按键状态跟踪（解决终端键盘重复延迟导致的"松手误判"）
    # getch() 在按键刚按下后的 ~500ms 内不会再捕获到，
    # 用 key_last_seen + 超时来避免这个间隙中被当成"松手"
    key_last_seen = {}

    stop_event = threading.Event()
    rx_thread = threading.Thread(
        target=rx_reader, args=(log_lines, log_win, lock, stop_event), daemon=True
    )
    rx_thread.start()

    last_log = time.time()

    try:
        while True:
            now = time.time()

            # -- 超时清除已松开的按键 --
            expired = [k for k, t in key_last_seen.items()
                       if now - t > KEY_TIMEOUT]
            for k in expired:
                del key_last_seen[k]

            # -- 键盘采集 (清空缓冲区) --
            stopped = False
            while True:
                k = stdscr.getch()
                if k == -1:
                    break
                if k == 27:  # ESC 退出
                    raise KeyboardInterrupt
                if k in (ord(' '),):
                    stopped = True
                if k in KEY_MAP:
                    key_last_seen[k] = now

            # -- 更新速度 (急停优先于任何按键) --
            if stopped:
                key_last_seen.clear()
                vx = vy = omega = 0.0
            elif key_last_seen:
                dx = dy = dw = 0
                for k in key_last_seen:
                    kx, ky, kw = KEY_MAP[k]
                    dx += kx
                    dy += ky
                    dw += kw
                # 平移归一化: 保证对角线方向不超速
                if dx != 0 or dy != 0:
                    mag = math.hypot(dx, dy)
                    vx = dx / mag * ROBOT_VEL_MAX
                    vy = dy / mag * ROBOT_VEL_MAX
                else:
                    vx = vy = 0.0
                omega = float(dw) * ROBOT_OMEGA_MAX
            else:
                # 无按键 → 立即停止（不滑行）
                # 如需滑行减速取消下面注释，注释掉 vx=vy=omega=0.0
                # vx *= DECAY; vy *= DECAY; omega *= DECAY
                # if abs(vx) < 0.5: vx = 0.0
                # if abs(vy) < 0.5: vy = 0.0
                # if abs(omega) < 0.5: omega = 0.0
                vx = vy = omega = 0.0

            # -- 发送控制指令 (每次循环即时发送) --
            do_log = (now - last_log >= STATUS_LOG_INTERVAL)
            speeds = compute_wheel_speeds(vx, vy, omega)
            send_velocity(speeds, log_lines, log_win, lock, do_log=do_log)
            if do_log:
                last_log = now

    except KeyboardInterrupt:
        pass
    finally:
        # 停止所有电机
        for mid in MOTOR_IDS.values():
            send_command(build_speed_cmd(mid, 0),
                         log_lines, log_win, lock, do_log=False)
        time.sleep(0.05)
        stop_event.set()
        rx_thread.join(timeout=1)
        ser.close()


if __name__ == '__main__':
    curses.wrapper(main)
