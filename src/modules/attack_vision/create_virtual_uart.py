#!/usr/bin/env python3
"""
创建虚拟串口对：使用 Python pty 模块替代 socat
这个脚本创建一个主从 PTY 对，并创建符号链接，然后保持运行以维持连接
"""

import pty
import os
import sys
import signal
import time
import select
import termios
import fcntl

def main():
    # 创建主从 PTY 对
    master_fd, slave_fd = pty.openpty()

    # 获取从设备名称
    slave_name = os.ttyname(slave_fd)
    master_name = os.ttyname(master_fd)

    print("=" * 60)
    print("创建虚拟串口对（替代 socat）")
    print("=" * 60)
    print(f"主设备 (master): {master_name}")
    print(f"从设备 (slave):  {slave_name}")

    # 创建符号链接
    link_tty = "/tmp/attack_vision_tty"
    link_src = "/tmp/attack_vision_src"

    # 删除旧链接（如果存在）
    try:
        os.unlink(link_tty)
    except:
        pass
    try:
        os.unlink(link_src)
    except:
        pass

    # 创建新链接
    # 注意：模块使用 slave，测试脚本通过 master_fd 写入
    os.symlink(slave_name, link_tty)  # 模块端（从设备）

    # 对于 master 端，我们创建一个命名管道（FIFO）来接收数据
    # 然后转发程序会从 FIFO 读取并写入 master_fd
    fifo_path = "/tmp/attack_vision_fifo"
    try:
        os.unlink(fifo_path)
    except:
        pass
    os.mkfifo(fifo_path, 0o666)

    print(f"\n符号链接已创建:")
    print(f"  {link_tty} -> {slave_name}")
    print(f"\n模块应打开: {link_tty}")
    print(f"测试脚本应写入: {fifo_path} (FIFO)")
    print(f"转发程序会将 FIFO 数据转发到 master_fd -> slave 端（模块读取）")

    print("\n保持连接运行中（按 Ctrl+C 停止）...")
    print("=" * 60)

    # 设置非阻塞
    fcntl.fcntl(master_fd, fcntl.F_SETFL, os.O_NONBLOCK)
    fcntl.fcntl(slave_fd, fcntl.F_SETFL, os.O_NONBLOCK)

    # 初始化 FIFO 文件描述符
    fifo_fd = None

    def signal_handler(sig, frame):
        print("\n\n停止虚拟串口...")
        try:
            if fifo_fd is not None:
                os.close(fifo_fd)
            os.close(master_fd)
            os.close(slave_fd)
        except:
            pass
        try:
            os.unlink(link_tty)
            os.unlink(fifo_path)
        except:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # 打开 FIFO（非阻塞模式，等待写入端）
    try:
        fifo_fd = os.open(fifo_path, os.O_RDWR | os.O_NONBLOCK)
        fcntl.fcntl(fifo_fd, fcntl.F_SETFL, os.O_NONBLOCK)
    except Exception as e:
        print(f"警告: 无法打开 FIFO: {e}")
        print("等待测试脚本连接...")

    # 数据转发循环
    byte_count = 0
    try:
        while True:
            # 检查可读的文件描述符
            check_fds = [master_fd, slave_fd]
            if fifo_fd is not None:
                check_fds.append(fifo_fd)

            rlist, _, _ = select.select(check_fds, [], [], 0.1)

            for fd in rlist:
                try:
                    data = os.read(fd, 1024)
                    if data:
                        byte_count += len(data)
                        # 转发数据
                        if fd == master_fd:
                            # master -> slave（模块读取）
                            os.write(slave_fd, data)
                            if byte_count <= 100:
                                print(f"[转发] master->slave: {len(data)} 字节")
                        elif fd == slave_fd:
                            # slave -> master（模块写入，如果有）
                            os.write(master_fd, data)
                            if byte_count <= 100:
                                print(f"[转发] slave->master: {len(data)} 字节")
                        elif fd == fifo_fd:
                            # FIFO -> master -> slave（测试脚本写入）
                            # 注意：PTY 的工作原理是：写入 master_fd 的数据会出现在 slave 端
                            # 所以模块（从 slave 设备读取）能收到写入 master_fd 的数据
                            written = os.write(master_fd, data)
                            if byte_count <= 100:
                                print(f"[转发] FIFO->master->slave: {len(data)} 字节 (written={written})")
                            # 确保数据立即刷新到设备
                            termios.tcdrain(master_fd)
                except (BlockingIOError, OSError) as e:
                    # 如果是 FIFO 还没有写入端，忽略错误
                    if fd != fifo_fd:
                        pass

            # 如果 FIFO 还没打开，尝试重新打开
            if fifo_fd is None:
                try:
                    fifo_fd = os.open(fifo_path, os.O_RDWR | os.O_NONBLOCK)
                    fcntl.fcntl(fifo_fd, fcntl.F_SETFL, os.O_NONBLOCK)
                    print(f"[连接] FIFO 已连接")
                except:
                    pass

            # 每1000字节输出一次统计
            if byte_count > 0 and byte_count % 1000 == 0:
                print(f"[统计] 已转发 {byte_count} 字节")

    except KeyboardInterrupt:
        pass
    finally:
        signal_handler(None, None)

if __name__ == "__main__":
    main()

