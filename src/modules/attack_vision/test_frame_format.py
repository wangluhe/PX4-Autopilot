#!/usr/bin/env python3
"""
测试工具：验证 attack_vision 帧格式是否正确
用于诊断帧校验失败问题
"""

import struct

def build_frame(locked: bool, pix_x: int, pix_y: int) -> bytes:
    """构建64字节帧（与README中的脚本一致）"""
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

def validate_frame_cpp_style(frame: bytes) -> tuple[bool, str]:
    """
    使用与C++代码相同的逻辑验证帧格式
    返回: (是否有效, 错误信息)
    """
    if len(frame) != 64:
        return False, f"长度错误: {len(frame)} != 64"

    # 检查帧头
    if frame[0] != 0xFC:
        return False, f"帧头0错误: 0x{frame[0]:02X} != 0xFC"
    if frame[1] != 0x2C:
        return False, f"帧头1错误: 0x{frame[1]:02X} != 0x2C"

    # 检查帧尾
    if frame[63] != 0xF0:
        return False, f"帧尾错误: 0x{frame[63]:02X} != 0xF0"

    # 异或校验：索引2~61异或，结果应等于索引62
    xor_val = 0
    for i in range(2, 62):  # 索引2到61（共60个字节）
        xor_val ^= frame[i]

    if xor_val != frame[62]:
        return False, f"异或校验失败: 计算值=0x{xor_val:02X}, 帧中值=0x{frame[62]:02X}"

    return True, "OK"

def parse_frame(frame: bytes) -> dict:
    """解析帧内容（与C++代码逻辑一致）"""
    status_5_6 = frame[4] | (frame[5] << 8)
    servo_state = frame[8]
    lock_bits = (status_5_6 >> 9) & 0x3
    locking = (lock_bits == 0x1) or (lock_bits == 0x2)
    exit_lock = (lock_bits == 0x3)

    pix_x = struct.unpack('<h', frame[58:60])[0]
    pix_y = struct.unpack('<h', frame[60:62])[0]

    lock_active = locking and (servo_state == 0x07)
    if exit_lock:
        lock_active = False

    return {
        'status_5_6': status_5_6,
        'servo_state': servo_state,
        'lock_bits': lock_bits,
        'locking': locking,
        'exit_lock': exit_lock,
        'lock_active': lock_active,
        'pix_x': pix_x,
        'pix_y': pix_y,
    }

def main():
    print("=== attack_vision 帧格式测试工具 ===\n")

    # 测试1: 锁定状态，有像素偏差
    print("测试1: 锁定状态，像素偏差(50, 30)")
    frame1 = build_frame(locked=True, pix_x=50, pix_y=30)
    valid, msg = validate_frame_cpp_style(frame1)
    print(f"  校验结果: {valid}, {msg}")
    if valid:
        parsed = parse_frame(frame1)
        print(f"  解析结果: lock_active={parsed['lock_active']}, pix=({parsed['pix_x']}, {parsed['pix_y']})")
        print(f"  详细信息: status_5_6=0x{parsed['status_5_6']:04X}, lock_bits=0x{parsed['lock_bits']:X}, servo=0x{parsed['servo_state']:02X}")
    print()

    # 测试2: 未锁定状态
    print("测试2: 未锁定状态，像素偏差(0, 0)")
    frame2 = build_frame(locked=False, pix_x=0, pix_y=0)
    valid, msg = validate_frame_cpp_style(frame2)
    print(f"  校验结果: {valid}, {msg}")
    if valid:
        parsed = parse_frame(frame2)
        print(f"  解析结果: lock_active={parsed['lock_active']}, pix=({parsed['pix_x']}, {parsed['pix_y']})")
    print()

    # 测试3: 负像素偏差
    print("测试3: 锁定状态，负像素偏差(-50, -30)")
    frame3 = build_frame(locked=True, pix_x=-50, pix_y=-30)
    valid, msg = validate_frame_cpp_style(frame3)
    print(f"  校验结果: {valid}, {msg}")
    if valid:
        parsed = parse_frame(frame3)
        print(f"  解析结果: lock_active={parsed['lock_active']}, pix=({parsed['pix_x']}, {parsed['pix_y']})")
    print()

    # 输出帧的十六进制（用于调试）
    print("测试1的帧内容（前16字节）:")
    print("  " + " ".join(f"{b:02X}" for b in frame1[:16]))
    print("测试1的帧内容（后16字节）:")
    print("  " + " ".join(f"{b:02X}" for b in frame1[48:64]))

if __name__ == "__main__":
    main()

