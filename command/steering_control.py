#!/usr/bin/env python3
"""
舵向角度控制 — PC 端简易角度闭环
=================================
基于 ticks 推算角度，P 控制器让转向电机转到目标角度。

用法: python3 steering_control.py --unit 1     # 控制 UNIT1
      python3 steering_control.py --unit 2     # 控制 UNIT2

数字命令直接设目标角度:
  90      → 转到 90°
  -45     → 转到 -45°（等价 315°）
  0       → 回正

其他命令:
  s 30    → 速度模式 speed=30
  a       → 切回角度模式
  stop    → 停止
  q       → 退出
"""

import sys, struct, time, threading, argparse
sys.path.insert(0, '/home/lin/ProjectRequirement/MCU/Lin_STM32/STM32_F103C8T6/STM32_Now/doc/ros2_can_bridge')
from mclm_can import MCLMCanInterface, DriveUnit

# ── 参数 ──
parser = argparse.ArgumentParser()
parser.add_argument('--unit', type=int, default=1, help='单元号 1~4')
args = parser.parse_args()

TARGET_UNIT = args.unit - 1          # 0-based
# 每单元独立标定值（实测）
TICKS_LIST = [9218, 9196, 7844, 9395]  # 实测于 2026-07-14
TICKS = TICKS_LIST[TARGET_UNIT]
STATUS_IDS = [0x321, 0x323, 0x325, 0x327]
CMD_IDS    = [0x121, 0x123, 0x125, 0x127]
STATUS_ID = STATUS_IDS[TARGET_UNIT]
KP = 0.8
MAX_SPEED = 60
DEAD_ZONE_DEG = 2.0

# ── 全局 ──
current_ticks = 0
abs_ticks = 0           # 解包后的连续绝对值（解决 uint16 回绕)
prev_raw = None         # 上一帧 raw，检测回绕用
zero_offset = 0         # 零位偏移（后面加 cal 命令）
target_angle = 0.0
angle_mode = True
manual_speed_val = 0
running = True

def unwrap_ticks(raw):
    """将 uint16 解包为连续 int32：检测 65535→0 或 0→65535 跳变"""
    global abs_ticks, prev_raw
    if prev_raw is None:
        abs_ticks = raw
    else:
        diff = raw - prev_raw
        if diff > 32768:       # 正向回绕: 65535 → 0
            diff -= 65536
        elif diff < -32768:    # 反向回绕: 0 → 65535
            diff += 65536
        abs_ticks += diff
    prev_raw = raw

def ticks_to_deg(t):
    return (t % TICKS) / TICKS * 360.0

def shortest_error(current_deg, target_deg):
    e = (target_deg - current_deg + 180) % 360 - 180
    return e

def cli_thread():
    """键盘输入线程"""
    global target_angle, angle_mode, manual_speed_val, running, zero_offset
    while running:
        try:
            line = input().strip().lower()
            if line == 'q':
                running = False
                break
            elif line == 'stop':
                print(">> 停止")
            elif line == 'a':
                angle_mode = True
                print(f">> 角度模式, 目标={target_angle:.0f}°")
            elif line == 'cal':
                zero_offset = abs_ticks
                target_angle = 0.0
                angle_mode = True
                print(f">> 已归零 (offset={zero_offset})")
            elif line.startswith('offset '):
                zero_offset = int(line[7:])
                print(f">> 设偏移={zero_offset}")
            elif line.startswith('s '):
                manual_speed_val = int(line[2:])
                angle_mode = False
                print(f">> 速度模式, speed={manual_speed_val}")
            else:
                target_angle = float(line) % 360
                angle_mode = True
                print(f">> 目标角度 = {target_angle:.0f}°")
        except (EOFError, ValueError):
            pass

def main():
    global current_ticks, abs_ticks, target_angle, angle_mode, manual_speed_val, running

    can = MCLMCanInterface(channel='can0')
    can.open()
    can.start()

    # 注册回调
    def on_raw(id, raw):
        global current_ticks
        if id == STATUS_ID and len(raw) >= 8:
            current_ticks = raw[2] | (raw[3] << 8)
            unwrap_ticks(current_ticks)
    can.on_raw(on_raw)
    time.sleep(0.3)

    # 启动输入线程
    t = threading.Thread(target=cli_thread, daemon=True)
    t.start()

    print("=" * 60)
    print(f"  舵向角度控制  UNIT{TARGET_UNIT+1}  (CAN 0x{STATUS_ID:03X})")
    print(f"  ticks/圈={TICKS}  Kp={KP}  max={MAX_SPEED}")
    print("=" * 60)
    print("  数字=设角度 | cal=当前位置归零 | s 30=速度 | stop | q")
    print("-" * 60)

    last_send = 0
    last_display = 0
    last_sent_speed = 999  # 去重用

    def angle_now():
        """基于连续 abs_ticks 计算当前角度，不受 uint16 回绕影响"""
        return ((abs_ticks - zero_offset) % TICKS) / TICKS * 360.0

    try:
        while running:
            now = time.time()
            angle = angle_now()

            # ── 控制计算 ──
            send_speed = None

            if angle_mode:
                err = shortest_error(angle, target_angle)
                if abs(err) < DEAD_ZONE_DEG:
                    send_speed = 0
                else:
                    speed = int(err * KP)
                    speed = max(-MAX_SPEED, min(MAX_SPEED, speed))
                    # 克服静摩擦
                    if abs(speed) < 6:
                        speed = 6 if speed > 0 else -6
                    send_speed = speed
            else:
                send_speed = int(manual_speed_val)
                if send_speed == 0:
                    send_speed = 0

            # ── 发 CAN 命令（去重，不重复发相同速度） ──
            if send_speed is not None and (send_speed != last_sent_speed or send_speed == 0):
                can.set_speed(DriveUnit(TARGET_UNIT), 'turn', send_speed)
                last_sent_speed = send_speed
                last_send = now

            # ── 显示 ──
            if now - last_display >= 0.2:
                err = shortest_error(angle, target_angle) if angle_mode else 0
                mode = 'A' if angle_mode else 'S'
                s = send_speed if send_speed is not None else last_sent_speed
                print(f"  [{mode}] 角度={angle:6.1f}°  目标={target_angle:6.1f}°  "
                      f"误差={err:+6.1f}°  speed={s:+3d}  abs={abs_ticks:6d}" + ' ' * 10,
                      end='\r')
                last_display = now

            time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        print("\n停止...")
        can.set_speed(DriveUnit(TARGET_UNIT), 'turn', 0)
        can.close()

if __name__ == '__main__':
    main()
