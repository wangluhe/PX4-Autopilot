#!/usr/bin/env python3
"""
简化版虚拟串口：只创建PTY对，不进行复杂的数据转发
"""

import pty
import os
import sys
import signal
import time

def main():
    # 创建主从 PTY 对
    master, slave = pty.openpty()

    # 获取设备名称
    slave_name = os.ttyname(slave)
    master_name = os.ttyname(master)

    print("=" * 60)
    print("创建简化版虚拟串口对")
    print("=" * 60)
    print(f"主设备 (master): {master_name}")
    print(f"从设备 (slave):  {slave_name}")

    # 创建符号链接
    link_tty = "/tmp/attack_vision_tty"

    # 删除旧链接
    try:
        os.unlink(link_tty)
    except:
        pass

    # 创建新链接
    os.symlink(slave_name, link_tty)
    print(f"\n符号链接: {link_tty} -> {slave_name}")

    # 创建FIFO用于测试
    fifo_path = "/tmp/attack_vision_fifo"
    try:
        os.unlink(fifo_path)
    except:
        pass
    os.mkfifo(fifo_path, 0o666)
    print(f"测试FIFO: {fifo_path}")

    print("\n虚拟串口已就绪")
    print("模块应打开: /tmp/attack_vision_tty")
    print("测试脚本应写入: /tmp/attack_vision_fifo")
    print("\n保持运行... (按 Ctrl+C 停止)")

    def signal_handler(sig, frame):
        print("\n\n停止虚拟串口...")
        try:
            os.close(master)
            os.close(slave)
            os.unlink(link_tty)
            os.unlink(fifo_path)
        except:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # 简单的数据转发
    try:
        # 打开FIFO（阻塞等待写入端）
        print("等待测试脚本连接FIFO...")
        fifo_fd = os.open(fifo_path, os.O_RDONLY | os.O_NONBLOCK)
        print("FIFO已连接")

        import select
        total_bytes = 0

        while True:
            rlist, _, _ = select.select([fifo_fd], [], [], 0.5)

            if fifo_fd in rlist:
                try:
                    data = os.read(fifo_fd, 1024)
                    if data:
                        # 写入master，数据会自动出现在slave端
                        written = os.write(master, data)
                        total_bytes += written
                        print(f"转发: {len(data)} 字节 -> slave (总计: {total_bytes} 字节)")
                except Exception as e:
                    print(f"转发错误: {e}")

    except Exception as e:
        print(f"错误: {e}")
    finally:
        signal_handler(None, None)

if __name__ == "__main__":
    main()
