#!/usr/bin/env python3
"""
测试串口连接：验证数据是否能从 /tmp/attack_vision_src 传输到 /tmp/attack_vision_tty
"""

import os
import time
import struct

def build_frame(locked: bool, pix_x: int, pix_y: int) -> bytes:
    """构建64字节帧"""
    data = bytearray(64)
    data[0] = 0xFC
    data[1] = 0x2C

    status = 0
    if locked:
        status |= (1 << 9)
    data[4:6] = struct.pack('<H', status)

    data[8] = 0x07 if locked else 0x00

    data[58:60] = struct.pack('<h', int(pix_x))
    data[60:62] = struct.pack('<h', int(pix_y))

    xor_val = 0
    for b in data[2:62]:
        xor_val ^= b
    data[62] = xor_val & 0xFF

    data[63] = 0xF0
    return bytes(data)

def main():
    DEV = "/tmp/attack_vision_fifo"  # 使用 FIFO 而不是设备

    # 检查 FIFO 是否存在
    if not os.path.exists(DEV):
        print(f"错误: FIFO {DEV} 不存在")
        print("请先运行: python3 create_virtual_uart.py")
        return

    print(f"打开 FIFO: {DEV}")
    try:
        # FIFO 需要以写入模式打开（会阻塞直到有读取端）
        fd = os.open(DEV, os.O_WRONLY)
        print(f"FIFO 打开成功，fd={fd}")
    except Exception as e:
        print(f"打开 FIFO 失败: {e}")
        return

    try:
        print("\n开始发送测试帧...")
        print("每帧64字节，帧头=0xFC 0x2C，帧尾=0xF0")
        print("按 Ctrl+C 停止\n")

        frame_count = 0
        while True:
            # 发送锁定帧
            frame = build_frame(locked=True, pix_x=50, pix_y=30)
            n = os.write(fd, frame)
            frame_count += 1

            if frame_count <= 5:
                print(f"发送第 {frame_count} 帧: {n} 字节")
                print(f"  帧头: 0x{frame[0]:02X} 0x{frame[1]:02X}")
                print(f"  帧尾: 0x{frame[63]:02X}")
                print(f"  校验: 0x{frame[62]:02X}")

            if frame_count % 25 == 0:
                print(f"已发送 {frame_count} 帧...")

            time.sleep(0.04)  # 25Hz (40ms间隔)

    except KeyboardInterrupt:
        print(f"\n\n停止发送，共发送 {frame_count} 帧")
    finally:
        os.close(fd)
        print("设备已关闭")

if __name__ == "__main__":
    main()

