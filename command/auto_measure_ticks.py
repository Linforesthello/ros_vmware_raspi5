#!/usr/bin/env python3
"""
自动测量：发低速转向命令 → 监听 ticks 变化
============================================
用法: python3 auto_measure_ticks.py
"""

import sys, struct, time
sys.path.insert(0, '/home/lin/ProjectRequirement/MCU/Lin_STM32/STM32_F103C8T6/STM32_Now/doc/ros2_can_bridge')
from mclm_can import MCLMCanInterface, DriveUnit

# 测试哪个单元 (0~3)
TARGET_UNIT = 0  # UNIT1

def main():
    can = MCLMCanInterface(channel='can0')
    can.open()
    can.start()

    turn_status_ids = [0x321, 0x323, 0x325, 0x327]
    status_id = turn_status_ids[TARGET_UNIT]

    print(f"UNIT{TARGET_UNIT+1} 转向电机 ticks 测量")
    print(f"监控 CAN ID: 0x{status_id:03X}")
    print("=" * 50)

    # 先静止观察 1 秒，记初始值
    print("\n[1] 静止观察 1 秒...")
    init_ticks = None
    t0 = time.time()
    while time.time() - t0 < 1.0:
        msg = can._bus.recv(timeout=0.3)
        if msg and msg.arbitration_id == status_id:
            init_ticks = msg.data[2] | (msg.data[3] << 8)
            print(f"  初始 ticks = {init_ticks}")

    if init_ticks is None:
        print("❌ 没收到状态帧，检查 CAN 连接")
        can.close()
        return

    # 发低速转向命令
    speed = 25
    print(f"\n[2] 发速度 = {speed} 让转向电机转...")
    can.set_speed(DriveUnit(TARGET_UNIT), 'turn', speed)

    # 观察 5 秒
    print(f"[3] 监听 5 秒 ticks 变化...")
    last_t = init_ticks
    t0 = time.time()
    samples = []
    while time.time() - t0 < 5.0:
        msg = can._bus.recv(timeout=0.3)
        if msg and msg.arbitration_id == status_id:
            now = time.time() - t0
            data = msg.data
            ticks = data[2] | (data[3] << 8)
            diff = ticks - last_t
            if diff > 32767: diff -= 65536
            if diff < -32767: diff += 65536
            print(f"  t={now:+.1f}s  ticks={ticks:5d}  diff={diff:+5d}  speed={struct.unpack_from('<h', data, 0)[0]:+4d}")
            samples.append((now, ticks))
            last_t = ticks

    # 停止
    print(f"\n[4] 停止电机...")
    can.stop_motor(DriveUnit(TARGET_UNIT), 'turn')
    time.sleep(0.3)

    # 分析
    if len(samples) >= 4:
        total_diff = samples[-1][1] - samples[0][1]
        if total_diff > 32767: total_diff -= 65536
        if total_diff < -32767: total_diff += 65536
        elapsed = samples[-1][0] - samples[0][0]
        print(f"\n{'=' * 50}")
        print(f"测量结果:")
        print(f"  5 秒内总变化: {total_diff} ticks")
        print(f"  平均速率: {total_diff/elapsed:.0f} ticks/秒")
        print(f"  编码器分辨率: 需要轮子转一圈时的 ticks 差值")
        print(f"  (建议在 SavvyCAN 标记位置后手动转一圈)")
    else:
        print("\n⚠️ 数据点不足")

    can.close()


if __name__ == '__main__':
    main()
