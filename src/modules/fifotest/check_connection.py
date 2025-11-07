#!/usr/bin/env python3
"""
诊断虚拟串口连接问题
"""

import os
import stat
import subprocess

def main():
    print("=== 虚拟串口连接诊断 ===")

    # 检查符号链接
    tty_path = "/tmp/attack_vision_tty"
    fifo_path = "/tmp/attack_vision_fifo"

    print(f"1. 检查符号链接: {tty_path}")
    if os.path.exists(tty_path):
        if os.path.islink(tty_path):
            target = os.readlink(tty_path)
            print(f"   ✓ 符号链接存在 -> {target}")

            # 检查目标设备
            if os.path.exists(target):
                print(f"   ✓ 目标设备存在: {target}")

                # 检查设备类型
                mode = os.stat(target).st_mode
                if stat.S_ISCHR(mode):
                    print(f"   ✓ 目标设备是字符设备")
                else:
                    print(f"   ✗ 目标设备不是字符设备")
            else:
                print(f"   ✗ 目标设备不存在: {target}")
        else:
            print(f"   ✗ 不是符号链接")
    else:
        print(f"   ✗ 符号链接不存在")

    print(f"\n2. 检查FIFO: {fifo_path}")
    if os.path.exists(fifo_path):
        mode = os.stat(fifo_path).st_mode
        if stat.S_ISFIFO(mode):
            print(f"   ✓ FIFO存在且类型正确")
        else:
            print(f"   ✗ 不是FIFO文件")
    else:
        print(f"   ✗ FIFO不存在")

    print(f"\n3. 检查进程:")
    # 检查虚拟串口服务
    result = subprocess.run(["pgrep", "-f", "virtual_uart_service"],
                          capture_output=True, text=True)
    if result.returncode == 0:
        print(f"   ✓ 虚拟串口服务运行中 (PID: {result.stdout.strip()})")
    else:
        print(f"   ✗ 虚拟串口服务未运行")

    # 检查PX4进程
    result = subprocess.run(["pgrep", "-f", "px4_sitl"],
                          capture_output=True, text=True)
    if result.returncode == 0:
        print(f"   ✓ PX4 SITL运行中 (PID: {result.stdout.strip()})")
    else:
        print(f"   ✗ PX4 SITL未运行")

    print(f"\n4. 检查文件描述符:")
    if os.path.exists(tty_path):
        try:
            result = subprocess.run(["lsof", tty_path],
                                  capture_output=True, text=True)
            if result.returncode == 0:
                print(f"   打开进程:")
                print(result.stdout)
            else:
                print(f"   无进程打开该设备")
        except Exception as e:
            print(f"   lsof检查失败: {e}")

    print(f"\n=== 诊断完成 ===")

if __name__ == "__main__":
    main()
