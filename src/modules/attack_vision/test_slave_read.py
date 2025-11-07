#!/usr/bin/env python3
"""
测试脚本：验证能否从 slave PTY 读取数据
"""

import os
import time

def main():
    slave_path = "/tmp/attack_vision_tty"

    if not os.path.exists(slave_path):
        print(f"错误: {slave_path} 不存在")
        print("请先运行: python3 create_virtual_uart.py")
        return

    # 检查是否是符号链接
    if os.path.islink(slave_path):
        real_path = os.readlink(slave_path)
        print(f"{slave_path} -> {real_path}")
    else:
        real_path = slave_path

    print(f"\n打开设备: {real_path}")
    try:
        fd = os.open(real_path, os.O_RDWR | os.O_NONBLOCK)
        print(f"打开成功，fd={fd}")
    except Exception as e:
        print(f"打开失败: {e}")
        return

    print("\n尝试读取数据（10秒）...")
    start_time = time.time()
    byte_count = 0

    try:
        while time.time() - start_time < 10:
            try:
                data = os.read(fd, 1024)
                if data:
                    byte_count += len(data)
                    print(f"读取到 {len(data)} 字节: {data[:16].hex()}")
                    if byte_count <= 100:
                        print(f"  前16字节: {data[:16]}")
            except BlockingIOError:
                time.sleep(0.01)
            except Exception as e:
                print(f"读取错误: {e}")
                break
    finally:
        os.close(fd)
        print(f"\n总共读取 {byte_count} 字节")

if __name__ == "__main__":
    main()

