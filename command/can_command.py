#!/usr/bin/env python3
"""
CanCmd — 设置 slcan CAN 接口，可选择波特率
  - RS00 电机默认 1Mbps → -s8
  - 通用 500kbps → -s6
"""
import glob
import subprocess
import sys
import time

def select_can_device():
    devices = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
    if not devices:
        print("错误: 未找到可用设备，请检查设备连接。")
        sys.exit(1)
    print("可用设备：")
    for i, p in enumerate(devices):
        print(f"  {i}: {p:20s}  ", end="")
    print()
    while True:
        try:
            idx_input = input(f"请输入设备编号 (0-{len(devices)-1}): ")
            idx = int(idx_input)
            if 0 <= idx < len(devices):
                return devices[idx]
        except (ValueError, IndexError):
            pass
        print("输入无效，请重试。")

def select_baud():
    """让用户选择波特率，返回 slcand 符号 (如 s8)"""
    print("\n选择 CAN 波特率：")
    print("  1) 1 Mbps    ← RS00 电机默认 (推荐)")
    print("  2) 500 kbps")
    print("  3) 250 kbps")
    print("  4) 125 kbps")
    choice = input("请输入 (1-4, 默认 1): ").strip()
    baud_map = {"1": ("1M", "s8"), "2": ("500k", "s6"),
                "3": ("250k", "s5"), "4": ("125k", "s4")}
    name, sym = baud_map.get(choice, ("1M", "s8"))
    print(f"  选择: {name} ({sym})")
    return sym

def setup_can_interface(device, can_interface_name="can0", baud_rate_symbol="s8"):
    print(f"\n正在为设备 {device} 设置 CAN 接口 {can_interface_name}...")
    print(f"波特率: {baud_rate_symbol}")

    subprocess.run(f"sudo ip link set {can_interface_name} down 2>/dev/null", shell=True)
    subprocess.run(f"sudo pkill -f 'slcand.*{can_interface_name}' 2>/dev/null", shell=True)
    time.sleep(0.5)

    slcand_command = f"sudo slcand -o -c -{baud_rate_symbol} {device} {can_interface_name}"
    print(f"  > {slcand_command}")
    result = subprocess.run(slcand_command, shell=True, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"  错误: 启动 slcand 失败。")
        print(f"  {result.stderr.strip()}")
        sys.exit(1)
    print(f"  slcand 已启动，等待接口就绪...")
    time.sleep(1)

    set_link_up_command = f"sudo ip link set {can_interface_name} up"
    print(f"  > {set_link_up_command}")
    result = subprocess.run(set_link_up_command, shell=True, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"  错误: 无法 'up' {can_interface_name} 接口。")
        print(f"  {result.stderr.strip()}")
        sys.exit(1)
    print(f"  接口 {can_interface_name} 已成功 'up'。")

    result = subprocess.run(f"ip link show {can_interface_name}", shell=True,
                           capture_output=True, text=True)
    if result.returncode == 0:
        print(f"\n--- {can_interface_name} 状态 ---")
        print(result.stdout.strip())
        print("--------------------")
    else:
        print(f"  警告: 无法获取 {can_interface_name} 的状态。")

    print(f"\nCAN 接口 {can_interface_name} 在设备 {device} 上设置成功。")

def launch_savvycan():
    print("\n正在启动 SavvyCAN...")
    subprocess.Popen(["./SavvyCAN"], cwd="/home/lin/SavvyCAN")
    print("SavvyCAN 已启动。")

if __name__ == "__main__":
    selected_device = select_can_device()
    baud = select_baud()
    setup_can_interface(selected_device, can_interface_name="can0", baud_rate_symbol=baud)
    launch_savvycan()
