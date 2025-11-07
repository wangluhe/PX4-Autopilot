#!/usr/bin/env python3
import os
import time
import struct

DEV = "/tmp/attack_vision_fifo"  # 使用 FIFO
HZ = 25.0

def build_frame(locked: bool, pix_x: int, pix_y: int) -> bytes:
    data = bytearray(64)
    data[0] = 0xFC
    data[1] = 0x2C

    status = 0
    if locked:
        status |= (1 << 9)  # Bit9=1 表示 01 锁定示例
    data[4:6] = struct.pack('<H', status)  # 第5-6字节，小端

    data[8] = 0x07 if locked else 0x00     # 第9字节：伺服状态

    data[58:60] = struct.pack('<h', int(pix_x))  # X: 第59-60字节
    data[60:62] = struct.pack('<h', int(pix_y))  # Y: 第61-62字节

    xor_val = 0
    for b in data[2:62]:
        xor_val ^= b
    data[62] = xor_val & 0xFF              # 校验：第63字节

    data[63] = 0xF0                        # 帧尾：第64字节
    return bytes(data)

def main():
    fd = os.open(DEV, os.O_WRONLY)  # FIFO 只需要写入模式
    try:
        t = 0.0
        dt = 1.0 / HZ
        while True:
            pix_x = 50 if (int(t) % 4) < 2 else -50
            pix_y = 30 if (int(t / 2) % 4) < 2 else -30
            locked = (int(t) % 10) < 5      # 每5秒锁定/未锁定切换
            frame = build_frame(locked, pix_x, pix_y)
            os.write(fd, frame)
            time.sleep(dt)
            t += dt
    finally:
        os.close(fd)

if __name__ == "__main__":
    main()
