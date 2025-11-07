#!/bin/bash

# 强制清理虚拟串口残留文件

echo "清理虚拟串口残留文件..."

# 删除符号链接
if [ -L "/tmp/attack_vision_tty" ]; then
    echo "删除符号链接: /tmp/attack_vision_tty"
    rm -f /tmp/attack_vision_tty
fi

# 删除FIFO
if [ -e "/tmp/attack_vision_fifo" ]; then
    echo "删除FIFO: /tmp/attack_vision_fifo"
    rm -f /tmp/attack_vision_fifo
fi

# 杀死可能残留的虚拟串口服务进程
pkill -f "virtual_uart_service.py"
echo "清理完成"
