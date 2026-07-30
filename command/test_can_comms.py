#!/usr/bin/env python3
"""
Step 1: 验证 Python CAN 通信 — 监听电机主动上报的状态帧 (20Hz)
======================================================================
MCLM_t2 每 50ms 主动上报状态帧 (0x321~0x328)，本脚本纯监听解码。

用法: python3 test_can_comms.py

预期输出（电机静止时）:
  [OK] CAN 接口 can0 已打开
  [10:00:01] ← UNIT1 turn   speed=+0 ticks=1234 pwm=0 target=+0 stall=0 sat=0
  [10:00:01] ← UNIT1 power  speed=+0 ticks=5678 pwm=0 target=+0 stall=0 sat=0
  [10:00:01] ← UNIT2 turn   speed=+0 ticks=...  ...
  每 50ms 刷新一次，持续 5 秒后总结
"""

import sys
sys.path.insert(0, '/home/lin/ProjectRequirement/MCU/Lin_STM32/STM32_F103C8T6/STM32_Now/doc/ros2_can_bridge')

from mclm_can import MCLMCanInterface, MotorStatus
import time

def main():
    print("=" * 60)
    print("CAN 通信验证 — 监听状态帧 (20Hz 主动上报)")
    print("=" * 60)

    can = MCLMCanInterface(channel='can0')
    can.open()
    can.start()
    print("[OK] CAN 接口 can0 已打开")
    print("[*] 正在监听 CAN 总线，5 秒后总结...\n")

    received = []

    @can.on_status
    def on_status(status: MotorStatus):
        ts = time.strftime('%H:%M:%S', time.localtime(status.timestamp))
        # 从 raw_data 标志位解析
        flags_str = ''
        if status.stall:     flags_str += ' STALL'
        if status.saturated: flags_str += ' SAT'
        line = (f"[{ts}] ← UNIT{status.unit_id+1} {status.motor_type:5s} "
                f"speed={status.current_speed:+3.0f} "
                f"pwm={status.pwm:5d} "
                f"target={status.target_speed:+3.0f}"
                f"{flags_str}")
        received.append(status)
        print(line)

    # 监听 5 秒
    time.sleep(5)

    # 总结
    print(f"\n{'=' * 60}")
    if received:
        # 按单元统计
        units_seen = set(s.unit_id for s in received)
        types_seen = set(s.motor_type for s in received)
        print(f"✅ CAN 通信正常！5 秒内收到 {len(received)} 帧")
        print(f"   可见单元: {sorted(units_seen)} ({'全部' if len(units_seen)==4 else '部分'})")
        print(f"   可见电机类型: {sorted(types_seen)}")
        print(f"\n   mclm_can.py 库工作正常，可以进入下一步开发。")
    else:
        print("⚠️  未收到任何状态帧")
        print("   可能原因:")
        print("   - CAN 适配器未正确连接")
        print("   - 电机控制器未上电")
        print("   - CAN 总线波特率不匹配 (当前 slcand 参数)")
        print("   - 总线缺少终端电阻")

    can.close()


if __name__ == '__main__':
    main()
