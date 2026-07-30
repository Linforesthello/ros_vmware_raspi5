#!/usr/bin/env python3
"""
测转向电机 ticks/圈（支持指定单元）
====================================
用法: python3 measure_steering_ticks.py --unit 1    # 测 UNIT1
      python3 measure_steering_ticks.py --unit 2    # 测 UNIT2

操作:
  1. 标记轮子朝向
  2. 按回车让电机低速转
  3. 轮子转一圈回到标记时按 Ctrl+C
  4. 看 ticks 变化量
"""

import sys, struct, time, argparse
sys.path.insert(0, '/home/lin/ProjectRequirement/MCU/Lin_STM32/STM32_F103C8T6/STM32_Now/doc/ros2_can_bridge')
from mclm_can import MCLMCanInterface, DriveUnit

parser = argparse.ArgumentParser()
parser.add_argument('--unit', type=int, default=1, help='单元号 1~4')
args = parser.parse_args()

TARGET_UNIT = args.unit - 1
STATUS_IDS = [0x321, 0x323, 0x325, 0x327]
STATUS_ID = STATUS_IDS[TARGET_UNIT]

can = MCLMCanInterface(channel='can0')
can.open()
can.start()

ticks_data = []

def on_raw(id, raw):
    if id == STATUS_ID and len(raw) >= 8:
        ticks = raw[2] | (raw[3] << 8)
        ticks_data.append((time.time(), ticks))
can.on_raw(on_raw)

print("=" * 60)
print(f"转向电机 ticks/圈测量 — UNIT{TARGET_UNIT+1}")
print("=" * 60)
print("\n准备:")
print(f"  1. 在 UNIT{TARGET_UNIT+1} 轮子上做个标记")
print("  2. 脚本会发速度=20 让转向电机转")
print("  3. 轮子转一圈回到标记时，按 Ctrl+C")
print("\n按回车开始...", end='', flush=True)
input()

time.sleep(0.5)
if len(ticks_data) < 2:
    print("❌ 没收到状态帧")
    can.close()
    exit(1)

init_ticks = ticks_data[-1][1]
print(f"\n初始 ticks = {init_ticks}")
print(f"\n发速度=20 给转向电机... 观察轮子，转一圈按 Ctrl+C\n")
print("实时 ticks:")
print("-" * 40)

can.set_speed(DriveUnit(TARGET_UNIT), 'turn', 20)

last_t = time.time()
try:
    while True:
        time.sleep(0.1)
        if ticks_data:
            _, t = ticks_data[-1]
            now = time.time()
            if now - last_t >= 0.2:
                total = t - init_ticks
                if total > 32767: total -= 65536
                if total < -32767: total += 65536
                print(f"  ticks={t:5d}  total={total:+5d}")
                last_t = now
except KeyboardInterrupt:
    print("\n" + "-" * 40)
    stop_ticks = ticks_data[-1][1]
    total = stop_ticks - init_ticks
    if total > 32767: total -= 65536
    if total < -32767: total += 65536

    print(f"\n结果:")
    print(f"  初始: {init_ticks}")
    print(f"  最终: {stop_ticks}")
    print(f"  变化: {abs(total)} ticks")
    if abs(total) > 0:
        print(f"  → UNIT{TARGET_UNIT+1} 每圈 ≈ {abs(total)} ticks")
        print(f"  → 每度 ≈ {abs(total)/360:.1f} ticks")
finally:
    can.stop_motor(DriveUnit(TARGET_UNIT), 'turn')
    time.sleep(0.3)
    can.close()
