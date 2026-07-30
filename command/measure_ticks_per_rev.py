#!/usr/bin/env python3
"""
测量编码器 ticks/圈 — 手动推车看实时 ticks
============================================
用法:
  1. python3 measure_ticks_per_rev.py
  2. 手动转动 UNIT1 的轮子（推车让那个轮子转）
  3. 观察 ticks 实时变化

不需要理解代码，看输出就行。
"""

import sys
sys.path.insert(0, '/home/lin/ProjectRequirement/MCU/Lin_STM32/STM32_F103C8T6/STM32_Now/doc/ros2_can_bridge')

from mclm_can import MCLMCanInterface
import time, struct

TARGET_UNIT = 0  # 0=UNIT1, 1=UNIT2, 2=UNIT3, 3=UNIT4

def main():
    can = MCLMCanInterface(channel='can0')
    can.open()
    can.start()
    print("=" * 60)
    print(f"轮子编码器 ticks 实时监测 — UNIT{TARGET_UNIT+1} 转向电机")
    print("=" * 60)
    print()
    print("📌 操作方法：")
    print(f"  1. 在地上用粉笔/胶带标记 UNIT{TARGET_UNIT+1} 轮子朝向")
    print(f"  2. 推车，让那个轮子转一圈回到标记位置")
    print(f"  3. 观察下面积累值变化")
    print()
    print("  或者发个速度命令让转向电机转：")
    print(f"    python3 -c \"")
    print(f"      import sys; sys.path.insert(0,'doc/ros2_can_bridge')")
    print(f"      from mclm_can import MCLMCanInterface, DriveUnit")
    print(f"      c = MCLMCanInterface(); c.open(); c.start()")
    print(f"      c.set_speed(DriveUnit.UNIT{TARGET_UNIT+1}, 'turn', 20)\"")
    print()
    print("按 Ctrl+C 退出")
    print("-" * 60)

    turn_status_ids = [0x321, 0x323, 0x325, 0x327]
    target_id = turn_status_ids[TARGET_UNIT]

    last_ticks = None
    seen = False

    while True:
        try:
            msg = can._bus.recv(timeout=0.3)
            if msg is None:
                continue
            if msg.arbitration_id != target_id:
                continue

            data = msg.data
            ticks = data[2] | (data[3] << 8)  # uint16 LE
            speed_raw = struct.unpack_from('<h', data, 0)[0]
            pwm = struct.unpack_from('<h', data, 4)[0]

            now = time.strftime('%H:%M:%S')

            if last_ticks is not None:
                diff = ticks - last_ticks
                if diff > 32767: diff -= 65536  # uint16 溢出处理
                if diff < -32767: diff += 65536
                print(f"[{now}]  ticks={ticks:5d}  diff={diff:+5d}  speed={speed_raw:+4d}  pwm={pwm}")
            else:
                print(f"[{now}]  ticks={ticks:5d}  (初始值)")

            last_ticks = ticks
            seen = True

        except KeyboardInterrupt:
            print(f"\n✅ 观察结束")
            print(f"轮子转一圈后 ticks 变化量 ≈ 每圈编码器计数")
            break
        except Exception:
            pass

    can.close()

if __name__ == '__main__':
    main()
