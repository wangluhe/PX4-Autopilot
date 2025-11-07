#!/usr/bin/env python3
"""
修复版虚拟串口服务：处理设备冲突问题
"""

import pty
import os
import sys
import signal
import time
import select
import logging
from pathlib import Path

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("virtual_uart")

class VirtualUARTService:
    def __init__(self):
        self.master_fd = None
        self.slave_fd = None
        self.slave_name = None
        self.master_name = None
        self.fifo_fd = None
        self.running = False

        # 路径配置
        self.link_path = Path("/tmp/attack_vision_tty")
        self.fifo_path = Path("/tmp/attack_vision_fifo")

    def cleanup_old_links(self):
        """清理旧的符号链接和FIFO"""
        try:
            # 检查符号链接是否存在且是否有效
            if self.link_path.exists():
                if self.link_path.is_symlink():
                    try:
                        target = self.link_path.readlink()
                        # 检查目标设备是否存在
                        if not Path(target).exists():
                            logger.warning(f"符号链接目标不存在: {target}，删除无效链接")
                            self.link_path.unlink()
                        else:
                            logger.info(f"发现现有符号链接: {self.link_path} -> {target}")
                    except Exception as e:
                        logger.warning(f"检查符号链接失败: {e}，删除无效链接")
                        self.link_path.unlink()
                else:
                    logger.warning(f"{self.link_path} 不是符号链接，删除")
                    self.link_path.unlink()

            # 清理FIFO
            if self.fifo_path.exists():
                self.fifo_path.unlink()
                logger.info("清理旧FIFO")

        except Exception as e:
            logger.error(f"清理旧链接时出错: {e}")

    def setup(self):
        """设置虚拟串口"""
        try:
            # 先清理旧链接
            self.cleanup_old_links()

            # 创建主从PTY对
            self.master_fd, self.slave_fd = pty.openpty()
            self.slave_name = os.ttyname(self.slave_fd)
            self.master_name = os.ttyname(self.master_fd)

            logger.info(f"创建虚拟串口对:")
            logger.info(f"  Master: {self.master_name}")
            logger.info(f"  Slave:  {self.slave_name}")

            # 确保目标设备存在
            if not Path(self.slave_name).exists():
                logger.error(f"从设备不存在: {self.slave_name}")
                return False

            # 创建新链接和FIFO
            self.link_path.symlink_to(self.slave_name)
            os.mkfifo(self.fifo_path, 0o666)

            # 验证符号链接是否创建成功
            if self.link_path.exists() and self.link_path.is_symlink():
                actual_target = self.link_path.readlink()
                logger.info(f"成功创建符号链接: {self.link_path} -> {actual_target}")

                # 验证链接是否指向正确的设备
                if str(actual_target) != self.slave_name:
                    logger.warning(f"符号链接目标不匹配: 期望={self.slave_name}, 实际={actual_target}")
            else:
                logger.error("符号链接创建失败")
                return False

            logger.info(f"创建FIFO: {self.fifo_path}")

            return True

        except Exception as e:
            logger.error(f"设置虚拟串口失败: {e}")
            # 发生错误时清理资源
            self.cleanup_resources()
            return False

    def cleanup_resources(self):
        """清理所有资源"""
        try:
            if self.master_fd:
                os.close(self.master_fd)
                self.master_fd = None
            if self.slave_fd:
                os.close(self.slave_fd)
                self.slave_fd = None
            if self.fifo_fd:
                os.close(self.fifo_fd)
                self.fifo_fd = None
        except Exception as e:
            logger.error(f"清理资源时出错: {e}")

    def start(self):
        """启动服务"""
        if not self.setup():
            logger.error("虚拟串口设置失败，无法启动服务")
            return False

        self.running = True

        # 信号处理
        def signal_handler(sig, frame):
            logger.info("接收到停止信号")
            self.stop()

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        logger.info("启动虚拟串口服务...")
        logger.info("PX4模块应打开: /tmp/attack_vision_tty")
        logger.info("数据源应写入: /tmp/attack_vision_fifo")

        # 打开FIFO（阻塞等待写入端连接）
        logger.info("等待测试脚本连接FIFO...")
        try:
            self.fifo_fd = os.open(self.fifo_path, os.O_RDONLY)  # 阻塞模式
            logger.info("FIFO已连接，开始转发数据...")
        except Exception as e:
            logger.error(f"打开FIFO失败: {e}")
            self.stop()
            return

        total_bytes = 0
        frame_count = 0
        last_stat_time = time.time()

        try:
            while self.running:
                # 读取FIFO数据
                try:
                    data = os.read(self.fifo_fd, 1024)
                    if data:
                        # 写入master，数据会自动出现在slave端
                        written = os.write(self.master_fd, data)
                        total_bytes += written
                        frame_count += len(data) // 64

                        # 每秒输出一次统计
                        current_time = time.time()
                        if current_time - last_stat_time >= 1.0:
                            logger.info(f"转发统计: {frame_count} 帧, {total_bytes} 字节")
                            last_stat_time = current_time

                except BlockingIOError:
                    # 非阻塞模式下没有数据，短暂休眠
                    time.sleep(0.01)
                except OSError as e:
                    if e.errno == 5:  # Input/output error (设备可能已关闭)
                        logger.error("设备I/O错误，可能设备已断开")
                        break
                    else:
                        logger.error(f"数据转发错误: {e}")
                        time.sleep(0.1)
                except Exception as e:
                    logger.error(f"未知错误: {e}")
                    time.sleep(0.1)

        except KeyboardInterrupt:
            logger.info("用户中断")
        finally:
            self.stop()

    def stop(self):
        """停止服务"""
        self.running = False
        logger.info("停止虚拟串口服务...")

        try:
            self.cleanup_resources()

            # 清理文件
            if self.link_path.exists():
                self.link_path.unlink()
                logger.info("删除符号链接")
            if self.fifo_path.exists():
                self.fifo_path.unlink()
                logger.info("删除FIFO")

            logger.info("清理完成")
        except Exception as e:
            logger.error(f"清理过程中出错: {e}")

def main():
    service = VirtualUARTService()
    service.start()

if __name__ == "__main__":
    main()
