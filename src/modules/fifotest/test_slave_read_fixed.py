#!/usr/bin/env python3
"""
修复版测试脚本：验证从 slave PTY 读取数据
"""

import os
import time
import select
import termios
import tty

def set_serial_params(fd):
    """设置串口参数"""
    # 获取当前终端设置
    old_settings = termios.tcgetattr(fd)

    try:
        # 设置原始模式
        tty.setraw(fd)

        # 获取新的终端设置
        new_settings = termios.tcgetattr(fd)

        # 设置波特率 (虽然虚拟串口不需要，但保持兼容)
        new_settings[4] = termios.B115200  # 输入波特率
        new_settings[5] = termios.B115200  # 输出波特率

        # 8N1 配置
        new_settings[2] &= ~termios.PARENB  # 无奇偶校验
        new_settings[2] &= ~termios.CSTOPB  # 1个停止位
        new_settings[2] &= ~termios.CSIZE   # 清除数据位设置
        new_settings[2] |= termios.CS8      # 8个数据位

        # 应用设置
        termios.tcsetattr(fd, termios.TCSANOW, new_settings)

        return old_settings
    except Exception as e:
        print(f"设置串口参数失败: {e}")
        return None

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
        # 以读写模式打开，不使用非阻塞
        fd = os.open(real_path, os.O_RDWR)
        print(f"打开成功，fd={fd}")
    except Exception as e:
        print(f"打开失败: {e}")
        return

    # 设置串口参数
    old_settings = set_serial_params(fd)

    print("\n尝试读取数据（15秒）...")
    print("等待数据...")

    start_time = time.time()
    byte_count = 0
    frame_count = 0

    try:
        while time.time() - start_time < 15:
            # 使用 select 检查是否有数据可读
            rlist, _, _ = select.select([fd], [], [], 0.1)

            if fd in rlist:
                try:
                    # 读取数据
                    data = os.read(fd, 1024)
                    if data:
                        byte_count += len(data)
                        frame_count += len(data) // 64  # 假设每帧64字节

                        print(f"\n读取到 {len(data)} 字节 (总: {byte_count} 字节, 约 {frame_count} 帧)")

                        # 显示前几个字节的十六进制
                        print(f"前16字节十六进制: {data[:16].hex()}")

                        # 尝试解析帧结构
                        if len(data) >= 64:
                            for i in range(0, min(len(data), 64), 64):
                                frame = data[i:i+64]
                                if len(frame) == 64:
                                    print(f"帧 {i//64 + 1}: 头={frame[0]:02X}{frame[1]:02X}, 尾={frame[63]:02X}")
                except Exception as e:
                    print(f"读取错误: {e}")
                    break
            else:
                # 没有数据时显示等待状态
                elapsed = time.time() - start_time
                if elapsed > 5 and byte_count == 0:
                    print(f"\r等待数据... {int(15 - elapsed)}秒剩余", end="", flush=True)

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        # 恢复终端设置
        if old_settings:
            termios.tcsetattr(fd, termios.TCSANOW, old_settings)

        os.close(fd)
        print(f"\n总共读取 {byte_count} 字节, 约 {frame_count} 帧")

if __name__ == "__main__":
    main()
