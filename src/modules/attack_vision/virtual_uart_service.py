#!/usr/bin/env python3
"""
增强版虚拟串口服务：改进资源清理和重启处理
"""

import pty
import os
import sys
import signal
import time
import select
import logging
import atexit
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
        self.initialized = False

        # 路径配置
        self.link_path = Path("/tmp/attack_vision_tty")
        self.fifo_path = Path("/tmp/attack_vision_fifo")

        # 注册退出处理
        atexit.register(self.emergency_cleanup)

    def emergency_cleanup(self):
        """紧急清理，在程序异常退出时调用"""
        if self.running:
            logger.warning("Emergency cleanup triggered")
            self.stop()

    def force_cleanup_system(self):
        """强制清理系统残留资源"""
        logger.info("执行强制系统清理...")

        # 清理符号链接
        if self.link_path.exists():
            try:
                if self.link_path.is_symlink():
                    self.link_path.unlink()
                    logger.info(f"删除符号链接: {self.link_path}")
                else:
                    logger.warning(f"{self.link_path} 不是符号链接，跳过删除")
            except Exception as e:
                logger.error(f"删除符号链接失败: {e}")

        # 清理FIFO
        if self.fifo_path.exists():
            try:
                self.fifo_path.unlink()
                logger.info(f"删除FIFO: {self.fifo_path}")
            except Exception as e:
                logger.error(f"删除FIFO失败: {e}")

        # 清理可能的僵尸进程
        try:
            import subprocess
            # 查找并杀死可能残留的虚拟串口服务进程
            result = subprocess.run(['pgrep', '-f', 'virtual_uart_service.py'],
                                  capture_output=True, text=True)
            if result.returncode == 0:
                pids = result.stdout.strip().split('\n')
                for pid in pids:
                    if pid and pid != str(os.getpid()):
                        logger.warning(f"杀死残留进程: {pid}")
                        subprocess.run(['kill', '-9', pid])
        except Exception as e:
            logger.error(f"清理僵尸进程失败: {e}")

        # 短暂等待确保资源释放
        time.sleep(0.5)

    def cleanup_old_links(self):
        """清理旧的符号链接和FIFO"""
        self.force_cleanup_system()

    def setup(self):
        """设置虚拟串口"""
        try:
            # 先强制清理旧链接
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
            try:
                self.link_path.symlink_to(self.slave_name)
                logger.info(f"创建符号链接: {self.link_path} -> {self.slave_name}")
            except FileExistsError:
                logger.warning("符号链接已存在，重新创建")
                self.link_path.unlink(missing_ok=True)
                self.link_path.symlink_to(self.slave_name)

            try:
                os.mkfifo(self.fifo_path, 0o666)
                logger.info(f"创建FIFO: {self.fifo_path}")
            except FileExistsError:
                logger.warning("FIFO已存在，使用现有FIFO")

            # 验证符号链接是否创建成功
            if self.link_path.exists() and self.link_path.is_symlink():
                actual_target = self.link_path.readlink()
                logger.info(f"符号链接验证: {self.link_path} -> {actual_target}")

                if str(actual_target) != self.slave_name:
                    logger.warning(f"符号链接目标不匹配: 期望={self.slave_name}, 实际={actual_target}")
                    # 重新创建正确的链接
                    self.link_path.unlink(missing_ok=True)
                    self.link_path.symlink_to(self.slave_name)
                    logger.info("已重新创建正确的符号链接")
            else:
                logger.error("符号链接创建失败")
                return False

            # 设置文件描述符为非阻塞
            import fcntl
            flags = fcntl.fcntl(self.master_fd, fcntl.F_GETFL)
            fcntl.fcntl(self.master_fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)

            self.initialized = True
            return True

        except Exception as e:
            logger.error(f"设置虚拟串口失败: {e}")
            import traceback
            logger.error(traceback.format_exc())
            self.cleanup_resources()
            return False

    def cleanup_resources(self):
        """清理所有资源"""
        logger.info("清理资源...")
        try:
            if self.master_fd is not None:
                os.close(self.master_fd)
                self.master_fd = None
                logger.debug("关闭 master_fd")

            if self.slave_fd is not None:
                os.close(self.slave_fd)
                self.slave_fd = None
                logger.debug("关闭 slave_fd")

            if self.fifo_fd is not None:
                os.close(self.fifo_fd)
                self.fifo_fd = None
                logger.debug("关闭 fifo_fd")

        except Exception as e:
            logger.error(f"清理资源时出错: {e}")

    def wait_for_fifo_connection(self, timeout=30):
        """等待FIFO连接，带超时机制"""
        logger.info(f"等待测试脚本连接FIFO (超时: {timeout}秒)...")

        start_time = time.time()
        while self.running and (time.time() - start_time) < timeout:
            try:
                # 尝试以非阻塞方式打开FIFO来检查是否有写入端
                test_fd = os.open(self.fifo_path, os.O_RDONLY | os.O_NONBLOCK)
                os.close(test_fd)

                # 如果有写入端，正式打开FIFO
                self.fifo_fd = os.open(self.fifo_path, os.O_RDONLY)
                logger.info("FIFO已连接，开始转发数据...")
                return True

            except (OSError, FileNotFoundError) as e:
                if not self.running:
                    break
                # 等待写入端连接
                time.sleep(0.5)

        logger.error(f"等待FIFO连接超时 ({timeout}秒)")
        return False

    def start(self):
        """启动服务"""
        logger.info("启动虚拟串口服务...")

        if not self.setup():
            logger.error("虚拟串口设置失败，无法启动服务")
            return False

        self.running = True

        # 信号处理
        def signal_handler(sig, frame):
            logger.info(f"接收到信号 {sig}，正在停止服务...")
            self.stop()

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        logger.info("虚拟串口服务初始化完成")
        logger.info(f"PX4模块应打开: {self.link_path}")
        logger.info(f"数据源应写入: {self.fifo_path}")

        # 等待FIFO连接
        if not self.wait_for_fifo_connection():
            self.stop()
            return False

        total_bytes = 0
        frame_count = 0
        last_stat_time = time.time()
        error_count = 0
        max_errors = 10

        try:
            while self.running:
                try:
                    # 读取FIFO数据
                    data = os.read(self.fifo_fd, 1024)
                    if data:
                        # 重置错误计数
                        error_count = 0

                        # 写入master，数据会自动出现在slave端
                        written = os.write(self.master_fd, data)
                        total_bytes += written
                        frame_count += len(data) // 64

                        # 每秒输出一次统计
                        current_time = time.time()
                        if current_time - last_stat_time >= 2.0:  # 改为2秒输出一次
                            logger.info(f"转发统计: {frame_count} 帧, {total_bytes} 字节, 错误计数: {error_count}")
                            last_stat_time = current_time

                except BlockingIOError:
                    # 非阻塞模式下没有数据是正常的
                    time.sleep(0.01)
                except OSError as e:
                    error_count += 1
                    if e.errno == 5:  # Input/output error
                        logger.error("设备I/O错误，可能设备已断开")
                        if error_count >= max_errors:
                            logger.error("达到最大错误计数，停止服务")
                            break
                    elif e.errno == 9:  # Bad file descriptor
                        logger.error("文件描述符错误，尝试恢复...")
                        self.recover_connection()
                    else:
                        logger.warning(f"数据转发错误 [{error_count}/{max_errors}]: {e}")

                    time.sleep(0.1)
                except Exception as e:
                    error_count += 1
                    logger.error(f"未知错误 [{error_count}/{max_errors}]: {e}")
                    if error_count >= max_errors:
                        logger.error("达到最大错误计数，停止服务")
                        break
                    time.sleep(0.1)

        except KeyboardInterrupt:
            logger.info("用户中断")
        except Exception as e:
            logger.error(f"服务运行异常: {e}")
            import traceback
            logger.error(traceback.format_exc())
        finally:
            self.stop()

    def recover_connection(self):
        """尝试恢复连接"""
        logger.info("尝试恢复连接...")
        self.cleanup_resources()
        time.sleep(1)

        if not self.setup():
            logger.error("连接恢复失败")
            return False

        return self.wait_for_fifo_connection(timeout=10)

    def stop(self):
        """停止服务"""
        if not self.running:
            return

        logger.info("停止虚拟串口服务...")
        self.running = False

        try:
            self.cleanup_resources()
            self.force_cleanup_system()
            logger.info("虚拟串口服务已完全停止")
        except Exception as e:
            logger.error(f"停止过程中出错: {e}")

def main():
    import argparse
    parser = argparse.ArgumentParser(description='虚拟串口服务')
    parser.add_argument('--clean', action='store_true', help='强制清理后退出')
    args = parser.parse_args()

    service = VirtualUARTService()

    if args.clean:
        logger.info("执行强制清理...")
        service.force_cleanup_system()
        return

    try:
        service.start()
    except Exception as e:
        logger.error(f"服务启动失败: {e}")
        import traceback
        logger.error(traceback.format_exc())
        service.force_cleanup_system()

if __name__ == "__main__":
    main()
