#!/usr/bin/env python3
"""
电机状态仪表盘 — 实时显示所有电机的 ticks 变化
================================================
用法: python3 dashboard.py
"""

import sys, struct, time
sys.path.insert(0, '/home/lin/ProjectRequirement/MCU/Lin_STM32/STM32_F103C8T6/STM32_Now/doc/ros2_can_bridge')
from mclm_can import MCLMCanInterface

# CAN ID 映射
MOTOR_MAP = {
    0x321: (0, '转向'), 0x322: (0, '驱动'),
    0x323: (1, '转向'), 0x324: (1, '驱动'),
    0x325: (2, '转向'), 0x326: (2, '驱动'),
    0x327: (3, '转向'), 0x328: (3, '驱动'),
}

# 数据存储: {(unit, type): (speed, ticks, pwm, target, stall, sat, timestamp)}
data = {}

def on_raw(id, raw, ts):
    if id not in MOTOR_MAP or len(raw) < 8:
        return
    unit, mtype = MOTOR_MAP[id]
    speed = struct.unpack_from('<h', raw, 0)[0]
    ticks = raw[2] | (raw[3] << 8)
    pwm   = struct.unpack_from('<h', raw, 4)[0]
    target = struct.unpack_from('<b', raw, 6)[0]
    flags = raw[7]
    data[(unit, mtype)] = (speed, ticks, pwm, target,
                           bool(flags&0x01), bool(flags&0x02), ts)

def show():
    now = time.time()
    print('\n' * 20, end='')  # 模拟清屏
    print("=" * 68)
    print("  舵轮底盘 CAN 状态  |  speed  ticks   pwm  target  状态")
    print("-" * 68)
    for u in range(4):
        line = f"UNIT{u+1}"
        for m in ('转向', '驱动'):
            key = (u, m)
            if key in data:
                s, t, pwm, target, stall, sat, ts = data[key]
                alive = (now - ts) < 0.5
                if alive:
                    flags = ''
                    if stall: flags += '⚠'
                    if sat: flags += '▲'
                    if not flags: flags = '✓'
                    line += f"  {m} {s:+3.0f} {t:5d} {pwm:4d} {target:+3.0f} {flags:3s}"
                else:
                    line += f"  {m}  离线   --   --   --   --"
            else:
                line += f"  {m}  --   --   --   --   --"
        print(line)
    print(f"\n  [{time.strftime('%H:%M:%S')}] Ctrl+C 退出")

can = MCLMCanInterface(channel='can0')
can.open()
can.start()

# 用 on_raw 而不是 on_status，因为 on_status 不传 ticks
can.on_raw(lambda id, raw: on_raw(id, raw, time.time()))

try:
    while True:
        time.sleep(0.5)
        show()
except KeyboardInterrupt:
    print("\n退出")
    can.close()
