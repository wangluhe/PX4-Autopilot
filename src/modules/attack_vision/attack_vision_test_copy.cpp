/**
 * @file attack_vision.cpp
 * @brief 图像末制导模块实现文件
 *
 * 功能说明：
 * 1. 通过串口接收吊舱64字节反馈帧（每40ms一次）
 * 2. 解析帧数据：锁定状态（第5-6字节Bit9~Bit10）和目标脱靶量（第59-62字节）
 * 3. 当锁定有效且数据新鲜时，切换到Offboard模式并发布速度控制指令
 * 4. 失锁或超时（200ms无数据）时，自动切换回悬停模式
 *
 * 安全机制：
 * - 帧校验（帧头、帧尾、异或校验）
 * - 超时保护（200ms无数据视为失效）
 * - 速度死区和限幅保护
 * - 参数开关控制（AV_EN）
 */

// 图像末制导，进入后无遥控器接管。
// 需要优化
#include "attack_vision_test.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/Limits.hpp>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <px4_platform_common/cli.h>
#include "vserial.h"  // 添加虚拟串口头文件
#include <commander/px4_custom_mode.h>

/**
 * @brief 构造函数
 * 初始化模块参数和工作队列
 */
/**
 * @brief 构造函数
 * 初始化模块参数和工作队列
 */
AttackVision::AttackVision()
	: ModuleParams(nullptr)
	, WorkItem(MODULE_NAME, px4::wq_configurations::ttyS5)
{
	// 根据编译环境显示不同的串口信息
	#ifdef __PX4_POSIX
		PX4_INFO("Using virtual serial port for SITL simulation");
	#else
		PX4_INFO("Using hardware UART port /dev/ttyS5");
	#endif
}

AttackVision::~AttackVision()
{
	close_uart();
}

void AttackVision::close_uart()
{
	#ifdef __PX4_POSIX
	// 虚拟串口不需要关闭操作，只需重置文件描述符
		_fd = -1;
	#else
	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}
	#endif
}

int AttackVision::task_spawn(int argc, char *argv[])
{
	PX4_INFO("=== Attack Vision task_spawn called ===");

	AttackVision *instance = new AttackVision();
	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		PX4_INFO("Scheduling AttackVision to work queue...");
		instance->ScheduleNow();

		PX4_INFO("AttackVision scheduled successfully");
		return PX4_OK;
	}
	PX4_ERR("alloc failed");
	return PX4_ERROR;
}

AttackVision *AttackVision::instantiate(int argc, char *argv[])
{
	return new AttackVision();
}

int AttackVision::custom_command(int argc, char *argv[])
{
	if (argc > 0 && strcmp(argv[0], "resume") == 0) {
		return resume_command();
	}

	if (argc > 0 && strcmp(argv[0], "kill") == 0) {
		return stop_command();
	}

	return print_usage("unknown command");
}

int AttackVision::soft_stop_command()
{
	AttackVision *instance = get_instance();

	if (!instance) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	instance->_guidance_paused = true;
	instance->_module_state = ModuleState::HOLD;
	instance->_allow_takeover = false;
	instance->_lock_active = false;
	instance->_pix_offset_x = 0;
	instance->_pix_offset_y = 0;
	PX4_INFO("attack_vision soft stop: 暂停末制导，保留OFFBOARD心跳");
	return PX4_OK;
}

int AttackVision::resume_command()
{
	AttackVision *instance = get_instance();

	if (!instance) {
		return PX4_ERROR;
	}

	instance->_guidance_paused = false;
	PX4_INFO("attack_vision resume: 允许末制导重新接管");
	return PX4_OK;
}

int AttackVision::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
		### Description

		图像末制导模块，通过串口接收吊舱反馈数据，实现基于图像的目标跟踪和制导功能。

		### 功能特性

		- 通过TELEM2串口接收吊舱64字节反馈帧（每40ms一次）
		- 解析锁定状态和目标脱靶量（像素偏差）
		- 锁定有效时自动切换到Offboard模式并发布速度控制指令
		- 失锁或超时时自动切换回悬停模式（安全保护）

		### 硬件连接

		吊舱串口连接到飞控TELEM2口（/dev/ttyS5）

		### 参数

		- AAATTKVIS_EN: 模块使能开关（0=禁用，1=启用）
		- AV_BAUD: 串口波特率（默认115200）
		- AV_KP: 速度控制增益（m/s每像素，默认0.001）
		- AV_DEAD: 像素死区阈值（默认5像素）
		- AV_MAX_V: 最大速度限制（m/s，默认1.0）

		)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("attack_vision", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "soft stop: 暂停末制导但保留OFFBOARD心跳");
	PRINT_MODULE_USAGE_COMMAND_DESCR("resume", "恢复允许末制导接管");
	PRINT_MODULE_USAGE_COMMAND_DESCR("kill", "真正退出attack_vision模块");
	PRINT_MODULE_USAGE_COMMAND("status");

	return 0;
}

int AttackVision::print_status()
{
	uint64_t now_us = hrt_absolute_time();
	uint64_t time_since_last_frame = now_us - _last_frame_time_us;

	vehicle_status_s vs{};
	bool has_status = _vehicle_status_sub.copy(&vs);

	PX4_INFO("=== Attack Vision Status ===");
	PX4_INFO("UART: %s", (_fd >= 0) ? "OPEN" : "CLOSED");
	PX4_INFO("Target Lock: %s", _lock_active ? "YES" : "NO");
	PX4_INFO("Pixel Offset: X=%d, Y=%d", (int)_pix_offset_x, (int)_pix_offset_y);
	PX4_INFO("Guidance Paused: %s", _guidance_paused ? "YES" : "NO");

	const char* state_str = "UNKNOWN";
	switch (_module_state) {
	case ModuleState::HOLD: state_str = "HOLD"; break;
	case ModuleState::SWITCHING_TO_OFFBOARD: state_str = "SWITCHING_TO_OFFBOARD"; break;
	case ModuleState::OFFBOARD: state_str = "OFFBOARD"; break;
	}
	PX4_INFO("Module State: %s", state_str);

	if (has_status) {
		PX4_INFO("Vehicle: nav_state=%d, arming_state=%d", vs.nav_state, vs.arming_state);
	}

	PX4_INFO("Last Frame: %.3f ms ago", (double)(time_since_last_frame) / 1000.0);

	return 0;
}

/**
 * @brief 配置串口参数
 * @param baudrate 波特率（9600, 19200, 38400, 57600, 115200）
 * @return true=成功，false=失败
 */
bool AttackVision::configure_uart(int baudrate)
{
	if (_fd < 0) { return false; }

	termios t{};
	if (tcgetattr(_fd, &t) != 0) {
		PX4_ERR("tcgetattr failed");
		close_uart(); // 添加资源清理
		return false;
	}
	cfmakeraw(&t);  // 设置为原始模式（无行缓冲、无回显等）
	t.c_cflag |= CLOCAL | CREAD;  // 本地连接，启用接收器

	// 根据参数设置波特率
	speed_t speed = B115200;
	if (baudrate == 9600) speed = B9600;
	else if (baudrate == 19200) speed = B19200;
	else if (baudrate == 38400) speed = B38400;
	else if (baudrate == 57600) speed = B57600;
	else if (baudrate == 115200) speed = B115200;
	else {
		PX4_ERR("Unsupported baudrate: %d", baudrate);
		close_uart(); // 添加资源清理
		return false;
	}

	cfsetispeed(&t, speed);
	cfsetospeed(&t, speed);
	if (tcsetattr(_fd, TCSANOW, &t) != 0) {
		PX4_ERR("tcsetattr failed");
		return false;
	}
	return true;
}

bool AttackVision::open_uart()
{
	#ifdef __PX4_POSIX
	// SITL/Posix 下使用虚拟串口
	PX4_INFO("Initializing virtual serial port for SITL simulation");

	// 初始化虚拟串口
	if (VSerial::get_instance().init() != 0) {
		PX4_ERR("failed to initialize virtual serial port");
		return false;
	}

	_fd = 1; // 虚拟文件描述符，用于标识串口已打开
	PX4_INFO("Virtual UART port opened successfully");
	return true;
	#else
	// 硬件板卡默认 TELEM2 口
	const char *dev = "/dev/ttyS5";

	PX4_INFO("Attempting to open hardware UART: %s", dev);

	_fd = ::open(dev, O_RDWR | O_NOCTTY);
	if (_fd < 0) {
		PX4_ERR("Failed to open %s: %s (errno=%d)", dev, strerror(errno), errno);
		return false;
	}

	PX4_INFO("Hardware UART opened successfully: %s, fd=%d", dev, _fd);

	// 配置串口
	if (!configure_uart(_param_av_baud.get())) {
		PX4_ERR("Failed to configure UART parameters");
		::close(_fd);
		_fd = -1;
		return false;
	}

	// 修复格式符问题
	PX4_INFO("UART configured with baudrate: %ld", _param_av_baud.get());
	return true;
	#endif
}



/**
 * @brief 校验帧格式
 * @param frame 64字节帧数据指针
 * @return true=校验通过，false=校验失败
 *
 * 校验内容：
 * 1. 帧头：第0字节=0xFC，第1字节=0x2C
 * 2. 帧尾：第63字节=0xF0
 * 3. 异或校验：第3~62字节异或结果应等于第63字节（实际是第62字节）
 */
bool AttackVision::validate_frame(const uint8_t *frame)
{
	if (!frame) return false;
	if (frame[0] != FRAME_HEAD_0 || frame[1] != FRAME_HEAD_1) return false;
	if (frame[63] != FRAME_TAIL) return false;

	// 异或校验：第3~62字节（索引2~61）异或，结果应等于第63字节（索引62）
	uint8_t xorv = 0;
	for (int i = 2; i <= 61; i++) { xorv ^= frame[i]; }
	return xorv == frame[62];
}

/**
 * @brief 尝试从串口读取一帧数据（适配poll模式）
 * @return true=成功读取并解析一帧，false=未完成或失败
 */
/**
 * @brief 尝试从串口读取一帧数据
 * @return true=成功读取并解析一帧，false=未完成或失败
 */
bool AttackVision::try_read_frame()
{
	#ifdef __PX4_POSIX
	// SITL模式：使用虚拟串口读取数据
	uint8_t buffer[64];
	int bytes_read = VSerial::get_instance().read(buffer, sizeof(buffer));

	if (bytes_read == sizeof(buffer)) {
		// 成功读取到完整帧
		memcpy(_buf, buffer, sizeof(buffer));
		_buf_len = sizeof(buffer);

		// 校验帧格式
		if (validate_frame(_buf)) {
		_last_frame_time_us = hrt_absolute_time();
		parse_frame_data();
		_buf_len = 0;  // 重置缓冲区

		return true;
		} else {
		PX4_WARN("虚拟串口帧校验失败");
		_buf_len = 0;
		}
	} else if (bytes_read > 0) {
		PX4_DEBUG("虚拟串口读取 %d 字节，期望 %zu", bytes_read, sizeof(buffer));
	}

	return false;
	#else
		// 硬件模式：统一使用滑动窗口处理
		if (_fd < 0) {
			PX4_ERR("文件描述符无效: %d", _fd);
			return false;
		}

		// 一次性读取所有可用数据
		uint8_t read_buf[256];
		ssize_t n = ::read(_fd, read_buf, sizeof(read_buf));

		if (n == 0) {
			// 没有数据可读（非阻塞模式正常）
			return false;
		} else if (n < 0) {
			if (errno == EAGAIN) {
			// 非阻塞模式下没有数据是正常的
			return false;
			} else {
			PX4_ERR("读取错误: %s", strerror(errno));
			return false;
			}
		}

		// 处理所有读取到的字节
		for (ssize_t i = 0; i < n; i++) {
			uint8_t byte = read_buf[i];

			// 如果缓冲区已满，滑动窗口
			if (_buf_len >= FRAME_LEN) {
			memmove(_buf, _buf + 1, FRAME_LEN - 1);
			_buf_len = FRAME_LEN - 1;
			}

			_buf[_buf_len++] = byte;

			// 检查是否收集到完整帧
			if (_buf_len >= FRAME_LEN) {
			// 校验帧格式
			bool ok = validate_frame(_buf);
			if (ok) {
				_last_frame_time_us = hrt_absolute_time();
				parse_frame_data();
				_buf_len = 0; // 统一重置缓冲区
				return true;
			} else {
				// 校验失败，滑动窗口继续搜索
				memmove(_buf, _buf + 1, FRAME_LEN - 1);
				_buf_len = FRAME_LEN - 1;
				PX4_WARN("帧校验失败，滑动窗口");
			}
			}
		}

		return false;
	#endif
}

/**
 * @brief 解析帧数据
 */
void AttackVision::parse_frame_data()
{
	// ========== 调试输出：打印完整帧数据 ==========
	static int debug_frame_count = 0;
	if (debug_frame_count < 5) {
		PX4_INFO("完整帧数据 (长度 %d 字节):", FRAME_LEN);
		for (int i = 0; i < FRAME_LEN; i++) {
			if (i % 16 == 0) {
			PX4_INFO("");  // 新行
			}
			PX4_INFO_RAW(" %02X", _buf[i]);  // 使用 PX4_INFO_RAW
		}
		PX4_INFO("");
		debug_frame_count++;
	}

	// ========== 解析关键字段 ==========
	// 第5-6字节：吊舱状态（UINT16，小端序）
	uint16_t status_5_6 = (uint16_t)_buf[4] | ((uint16_t)_buf[5] << 8);
	// 第9字节：伺服状态
	uint8_t servo_state = _buf[8];

	// 特别注意Bit9和Bit10（协议中的锁定状态位）
	bool bit9 = (status_5_6 & (1 << 9)) != 0;
	bool bit10 = (status_5_6 & (1 << 10)) != 0;

	// 根据协议第14页，Bit9~Bit10表示目标锁定标识位：
	// 00: 默认（无效）
	// 01: 锁定中
	// 10: 锁定预测
	// 11: 退出锁定
	int lock_state = ((bit10 ? 1 : 0) << 1) | (bit9 ? 1 : 0);
	// const char* lock_state_str = "";
	// switch (lock_state) {
	// 	case 0: lock_state_str = "00-默认(无效)"; break;
	// 	case 1: lock_state_str = "01-锁定中"; break;
	// 	case 2: lock_state_str = "10-锁定预测"; break;
	// 	case 3: lock_state_str = "11-退出锁定"; break;
	// }
	// PX4_INFO("锁定标识: %s", lock_state_str);

	// 锁定有效条件：锁定标识位为01（锁定中）且伺服状态为跟踪模式（0x07）
	// bool locking = (lock_state == 1);  // 01状态表示锁定中
	bool locking = (lock_state == 1) || (lock_state == 2);  // 01(锁定中) 或 10(锁定预测)

	// 调试输出伺服状态
	// const char* servo_state_str = "";
	// switch (servo_state) {
	// 	case 0x01: servo_state_str = "载荷关"; break;
	// 	case 0x02: servo_state_str = "手动"; break;
	// 	case 0x03: servo_state_str = "收藏"; break;
	// 	case 0x04: servo_state_str = "数引"; break;
	// 	case 0x05: servo_state_str = "航向锁定"; break;
	// 	case 0x06: servo_state_str = "扫描"; break;
	// 	case 0x07: servo_state_str = "跟踪"; break;
	// 	case 0x08: servo_state_str = "垂直下视"; break;
	// 	case 0x09: servo_state_str = "陀螺自动较漂"; break;
	// 	case 0x0A: servo_state_str = "陀螺温度较漂"; break;
	// 	case 0x0B: servo_state_str = "航向随动"; break;
	// 	case 0x0C: servo_state_str = "归中"; break;
	// 	case 0x0D: servo_state_str = "手动陀螺较漂"; break;
	// 	case 0x0E: servo_state_str = "姿态指引"; break;
	// 	default: servo_state_str = "未知"; break;
	// }
	// PX4_INFO("伺服状态: 0x%02X (%s)", servo_state, servo_state_str);

	_lock_active = locking && (servo_state == 0x07);

	// ========== 原脱靶量解析（保留，兼容扩展） ==========

	// 第59-60字节：目标脱靶量-方位方向（INT16，小端序，单位：像素）
	_pix_offset_x = (int16_t)((uint16_t)_buf[58] | ((uint16_t)_buf[59] << 8));
	// 第61-62字节：目标脱靶量-俯仰方向（INT16，小端序，单位：像素）
	_pix_offset_y = (int16_t)((uint16_t)_buf[60] | ((uint16_t)_buf[61] << 8));
	// 调试输出脱靶量
	// PX4_INFO("[58]=0x%02X, [59]=0x%02X, [60]=0x%02X, [61]=0x%02X",
	// 	_buf[58], _buf[59], _buf[60], _buf[61]);
	// PX4_INFO("=========================================");

	// ========== 新增：解析吊舱姿态角（关键修改） ==========
	// 假设字节位置（需根据实际协议调整！）：
	// 字节10-11：方位角（INT16，0.01°/LSB，小端序）
	// 字节12-13：俯仰角（INT16，0.01°/LSB，小端序）
	// 字节14-15：滚转角（INT16，0.01°/LSB，小端序）
	roll_deg_100 = (int16_t)((uint16_t)_buf[13] | ((uint16_t)_buf[14] << 8));
	pitch_deg_100 = (int16_t)((uint16_t)_buf[11] | ((uint16_t)_buf[12] << 8));
	yaw_deg_100 = (int16_t)((uint16_t)_buf[9] | ((uint16_t)_buf[10] << 8));
	// 转换为弧度（PX4姿态控制单位）
	_gimbal_roll = (roll_deg_100 / 100.0f) * M_PI_F / 180.0f;
	_gimbal_pitch = (pitch_deg_100 / 100.0f) * M_PI_F / 180.0f;
	_gimbal_yaw = (yaw_deg_100 / 100.0f) * M_PI_F / 180.0f;

	// 打印吊舱姿态（调试用）
	// PX4_INFO("吊舱姿态: 横滚=%.2f° (%.3frad), 俯仰=%.2f° (%.3frad), 方位=%.2f° (%.3frad)",
	// 	(double)roll_deg_100 / 100.0, (double)_gimbal_roll,
	// 	(double)pitch_deg_100 / 100.0, (double)_gimbal_pitch,
	// 	(double)yaw_deg_100 / 100.0, (double)_gimbal_yaw);
	// PX4_INFO("=========================================");
}



/**
 * @brief 切换到Offboard模式
 * @return true=已在Offboard模式且已解锁，false=正在切换中或未解锁
 *
 * 注意：需要飞控已解锁且允许Offboard模式（通过地面站参数设置）
 */
bool AttackVision::switch_to_offboard()
{
	// 增加RC授权检查：仅当RC在Offboard档位时才切换
	if (_current_rc_mode != RCMode::MODE_OFFBOARD) {
		PX4_WARN("RC not in Offboard mode, skip switch");
		return false;
	}

	vehicle_status_s vs{};
	if (!_vehicle_status_sub.copy(&vs)) {
		PX4_WARN("无法获取vehicle_status");
		return false;
	}

	PX4_INFO("当前状态: nav_state=%d, arming_state=%d", vs.nav_state, vs.arming_state);

	// 检查是否已解锁
	if (vs.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		PX4_WARN("飞控未解锁，无法切换到Offboard模式");
		return false;
	}

	// 如果已经在Offboard模式
	if (vs.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
		PX4_INFO("已在Offboard模式");
		return true;
	}

	// 限制命令发布频率
	uint64_t now = hrt_absolute_time();
	if (now - _last_cmd_publish_time < MIN_CMD_INTERVAL_US) {
		return false;
	}

	PX4_INFO("准备切换到Offboard模式（RC授权），先发布控制信号...");

	// 短暂延迟确保控制信号被接收
	usleep(100000); // 100ms

	// 发送模式切换命令
	vehicle_command_s cmd{};
	cmd.timestamp = now;
	cmd.param1 = (float)1;  // 主模式
	cmd.param2 = (float)6;  // PX4_CUSTOM_MAIN_MODE_OFFBOARD
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	cmd.target_system = 1;
	cmd.target_component = 1;
	cmd.source_system = 1;
	cmd.source_component = 1;
	cmd.confirmation = 0;
	cmd.from_external = false;

	_vehicle_cmd_pub.publish(cmd);
	_last_cmd_publish_time = now;

	PX4_INFO("已发送切换到Offboard模式命令");
	return false;
}

// 新增：解析RC通道的PWM值，判断当前模式档位
AttackVision::RCMode AttackVision::parse_rc_mode()
{
	input_rc_s rc{};
	if (!_rc_input_sub.copy(&rc)) {
		PX4_WARN("Failed to read RC input");
		return RCMode::MODE_UNKNOWN;
	}

	// 检查RC信号是否有效
	if (rc.timestamp_last_signal == 0 || (hrt_absolute_time() - rc.timestamp_last_signal) > 1000000) {
		PX4_WARN("RC signal lost");
		return RCMode::MODE_UNKNOWN;
	}

	// 获取模式通道的PWM值（RC通道索引从0开始，CH6对应索引5）
	int ch_idx = _rc_mode_channel - 1;
	if (ch_idx < 0 || ch_idx >= 18) {
		PX4_ERR("Invalid RC mode channel: %d", _rc_mode_channel);
		return RCMode::MODE_UNKNOWN;
	}

	int pwm = rc.values[ch_idx];
	// 限幅PWM值
	pwm = math::constrain(pwm, RC_PWM_MIN, RC_PWM_MAX);

	// 判断档位
	if (pwm >= RC_THRESHOLD_OFFBOARD_LOW && pwm <= RC_THRESHOLD_OFFBOARD_HIGH) {
		return RCMode::MODE_OFFBOARD;
	} else if (pwm >= RC_THRESHOLD_POSITION_LOW && pwm < RC_THRESHOLD_POSITION_HIGH) {
		return RCMode::MODE_POSITION;
	} else if (pwm >= RC_THRESHOLD_ALTITUDE_LOW && pwm < RC_THRESHOLD_ALTITUDE_HIGH) {
		return RCMode::MODE_ALTITUDE;
	} else {
		return RCMode::MODE_UNKNOWN;
	}
}

// 新增：根据RC模式切换飞控模式
void AttackVision::switch_to_rc_mode(RCMode target_mode)
{
	uint64_t now = hrt_absolute_time();
	if (now - _last_cmd_publish_time < MIN_CMD_INTERVAL_US) {
		return;
	}

	vehicle_command_s cmd{};
	cmd.timestamp = now;
	cmd.target_system = 1;
	cmd.target_component = 1;
	cmd.source_system = 1;
	cmd.source_component = 1;
	cmd.confirmation = 0;
	cmd.from_external = false;
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;

	switch (target_mode) {
		case RCMode::MODE_POSITION:
		// 切换到Position模式（定点）
		cmd.param1 = 1;  // 主模式：MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
		cmd.param2 = PX4_CUSTOM_MAIN_MODE_POSCTL;  // 子模式：POSITION
		PX4_INFO("Switch to POSITION mode (RC command)");
		break;

		case RCMode::MODE_ALTITUDE:
		// 切换到Altitude模式（定高）
		cmd.param1 = 1;  // 主模式：MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
		cmd.param2 = PX4_CUSTOM_MAIN_MODE_ALTCTL;  // 子模式：ALTITUDE
		PX4_INFO("Switch to ALTITUDE mode (RC command)");
		break;

		case RCMode::MODE_UNKNOWN:
		// 未知档位，切换到悬停模式
		cmd.param1 = 1;  // 主模式：AUTO
		cmd.param2 = PX4_CUSTOM_MAIN_MODE_AUTO;  // 主模式 AUTO
		cmd.param3 = PX4_CUSTOM_SUB_MODE_AUTO_LOITER;  // 子模式：AUTO_LOITER
		PX4_INFO("Switch to LOITER mode (unknown RC mode)");
		break;

		default:
		return;
	}

	_vehicle_cmd_pub.publish(cmd);
	_last_cmd_publish_time = now;
}


/**
 * @brief 切换到悬停模式（Auto Loiter）
 *
 * 当失锁或超时时，自动切换回悬停模式，确保飞行安全
 */

void AttackVision::switch_to_hold()
{
	uint64_t now = hrt_absolute_time();
	if (now - _last_cmd_publish_time < MIN_CMD_INTERVAL_US) {
		return;
	}

	vehicle_command_s cmd{};
	cmd.timestamp = now;
	cmd.param1 = (float)1;
	cmd.param2 = PX4_CUSTOM_MAIN_MODE_AUTO;
	cmd.param3 = PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	cmd.target_system = 1;
	cmd.target_component = 1;
	cmd.source_system = 1;
	cmd.source_component = 1;
	cmd.confirmation = false;
	cmd.from_external = false;

	_vehicle_cmd_pub.publish(cmd);
	_last_cmd_publish_time = now;

	PX4_INFO("已发送切换到悬停模式命令");
}

void AttackVision::reset_guidance_state()
{
	_module_state = ModuleState::HOLD;
	_switch_start_time = 0;
	_allow_takeover = false;
	_external_mission_active = false;
	_lock_active = false;
	_pix_offset_x = 0;
	_pix_offset_y = 0;
	_buf_len = 0;
	_last_frame_time_us = 0;
}

void AttackVision::safe_stop_guidance()
{
	external_mission_active_s external_mission{};
	if (_external_mission_active_sub.copy(&external_mission)) {
		_external_mission_active = external_mission.external_mission_active;
	}

	vehicle_status_s vehicle_status{};
	const bool has_vehicle_status = _vehicle_status_sub.copy(&vehicle_status);
	const bool vehicle_armed = has_vehicle_status && (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	const bool vehicle_in_offboard = has_vehicle_status && (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	PX4_INFO("attack_vision stopping: state=%d, armed=%d, offboard=%d, external_active=%d",
		(int)_module_state,
		(int)vehicle_armed,
		(int)vehicle_in_offboard,
		(int)_external_mission_active);

	if (vehicle_armed && vehicle_in_offboard) {
		for (int i = 0; i < 5; ++i) {
			publish_offboard_velocity(0.0f, 0.0f, 0.0f, 0.0f);
			usleep(20000);
		}

		if (_external_mission_active) {
			PX4_INFO("上位机任务已接管，attack_vision停止发布控制量");
		} else {
			#ifdef __PX4_POSIX
				PX4_INFO("仿真模式：未收到external_mission_active，attack_vision stop保持OFFBOARD交由上位机接管");
				for (int i = 0; i < 150; ++i) {
					vehicle_status_s latest_status{};
					const bool latest_status_ok = _vehicle_status_sub.copy(&latest_status);
					const bool still_in_offboard = latest_status_ok &&
						(latest_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

					if (!still_in_offboard) {
						PX4_WARN("仿真模式：等待上位机接管期间飞控已退出OFFBOARD");
						break;
					}

					publish_offboard_velocity(0.0f, 0.0f, 0.0f, 0.0f);
					usleep(20000);
				}
			#else
				for (int i = 0; i < 10; ++i) {
					vehicle_status_s latest_status{};
					const bool latest_status_ok = _vehicle_status_sub.copy(&latest_status);
					const bool still_in_offboard = latest_status_ok &&
						(latest_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

					if (!still_in_offboard) {
						break;
					}

					publish_offboard_velocity(0.0f, 0.0f, 0.0f, 0.0f);
					_last_cmd_publish_time = 0;
					switch_to_hold();
					usleep(100000);
				}

				PX4_INFO("attack_vision stop: Offboard已释放到AUTO_LOITER悬停");
			#endif
		}
	}

	reset_guidance_state();
}

/**
 * @brief 发布Offboard速度控制指令
 * @param vx 前向速度（m/s，机体系）
 * @param vy 横向速度（m/s，机体系）
 * @param vz 垂直速度（m/s，向上为正）
 * @param yaw_rate 偏航角速度（rad/s）
 *
 * 需要同时发布offboard_control_mode和trajectory_setpoint两个话题
 */
void AttackVision::publish_position_offboard_heartbeat()
{
	offboard_control_mode_s ocm{};
	ocm.timestamp = hrt_absolute_time();
	ocm.position = true;
	ocm.velocity = false;
	ocm.acceleration = false;
	ocm.attitude = false;
	ocm.body_rate = false;
	ocm.thrust_and_torque = false;
	ocm.direct_actuator = false;
	_offboard_ctrl_pub.publish(ocm);
}

void AttackVision::publish_offboard_velocity(float vx, float vy, float vz, float yaw_rate)
{

	// 发布Offboard控制模式：仅使用速度控制
	offboard_control_mode_s ocm{};
	ocm.timestamp = hrt_absolute_time();
	ocm.position = false;
	ocm.velocity = true;  // 启用速度控制
	ocm.acceleration = false;
	ocm.attitude = false;
	ocm.body_rate = false;
	ocm.thrust_and_torque = false;
	ocm.direct_actuator = false;
	_offboard_ctrl_pub.publish(ocm);

	// 发布速度设定值（NED坐标系）
	trajectory_setpoint_s sp{};
	sp.timestamp = ocm.timestamp;

	// 初始化所有字段为NaN（表示不控制）
	sp.position[0] = NAN;
	sp.position[1] = NAN;
	sp.position[2] = NAN;
	sp.velocity[0] = NAN;
	sp.velocity[1] = NAN;
	sp.velocity[2] = NAN;
	sp.acceleration[0] = NAN;
	sp.acceleration[1] = NAN;
	sp.acceleration[2] = NAN;
	sp.yaw = NAN;
	sp.yawspeed = NAN;

	// 关键修复：设置速度控制（NED坐标系）
	// 注意：NED坐标系中，向下为正，所以要保持高度需要 velocity[2] = 0
	sp.velocity[0] = vx;  // North方向速度（前向）
	sp.velocity[1] = vy;  // East方向速度（右侧）
	sp.velocity[2] = vz; //

	_traj_sp_pub.publish(sp);

	PX4_DEBUG("发布速度控制: vx=%.2f, vy=%.2f, vz=%.2f", (double)vx, (double)vy, (double)0.0f);

	#ifdef __PX4_POSIX
		VSerial::get_instance().set_control_input(vx, vy, vz);  // 传入三个速度分量

		// 调试：获取当前脱靶量状态
		int16_t current_offset_x, current_offset_y;
		VSerial::get_instance().get_target_offset(current_offset_x, current_offset_y);
	#endif

}

// 工具函数：四元数转欧拉角（FRD->NED，单位：弧度）
// q: [w, x, y, z] 四元数
// 返回：roll(横滚), pitch(俯仰), yaw(偏航)
void quaternion_to_euler(const float q[4], float &roll, float &pitch, float &yaw)
{
	// 提取四元数分量
	const float qw = q[0];
	const float qx = q[1];
	const float qy = q[2];
	const float qz = q[3];

	// 四元数转欧拉角（PX4 FRD body -> NED earth 坐标系）
	// 横滚 (roll)：绕X轴旋转
	roll = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy));
	// 俯仰 (pitch)：绕Y轴旋转（限幅避免万向锁）
	pitch = asinf(math::constrain(2.0f * (qw * qy - qz * qx), -1.0f, 1.0f));
	// 偏航 (yaw)：绕Z轴旋转（归一化到 [-π, π]）
	yaw = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
}



void AttackVision::handle_guidance()
{
	if (!check_guidance_ready()) {
		return;
	}

	VehicleGuidanceState veh{};
	if (!read_vehicle_guidance_state(veh)) {
		return;
	}

	GimbalNedPose gimbal{};
	if (!get_gimbal_ned_pose(veh.q_veh_ned, gimbal)) {
		return;
	}

	GuidanceCommand cmd{};
	if (!build_guidance_command(veh, gimbal, cmd)) {
		return;
	}

	publish_guidance_command(cmd);
}

bool AttackVision::check_guidance_ready()
{
	if (!_allow_takeover) {
		return false;
	}

	const float att_kp = _param_av_att_kp.get();
	const float max_forward_v = _param_av_forward_v.get();

	if (!PX4_ISFINITE(att_kp) || !PX4_ISFINITE(max_forward_v)) {
		PX4_ERR("guidance params invalid");
		return false;
	}

	return true;
}

bool AttackVision::read_vehicle_guidance_state(VehicleGuidanceState &state)
{
	vehicle_attitude_s veh_att{};
	if (!_vehicle_attitude_sub.copy(&veh_att)) {
		PX4_WARN("no vehicle attitude");
		return false;
	}

	state.q_veh_ned = matrix::Quatf(veh_att.q);
	matrix::Eulerf euler_veh_ned(state.q_veh_ned);
	state.veh_roll = euler_veh_ned.phi();
	state.veh_pitch = euler_veh_ned.theta();
	state.veh_yaw = euler_veh_ned.psi();
	state.valid = true;
	return true;
}

bool AttackVision::get_gimbal_ned_pose(const matrix::Quatf &q_veh_ned, GimbalNedPose &pose)
{
	#ifdef __PX4_POSIX
		pose.gimbal_roll_ned = _fixed_gimbal_roll_ned;
		pose.gimbal_pitch_ned = _fixed_gimbal_pitch_ned;
		pose.gimbal_yaw_ned = _fixed_gimbal_yaw_ned;
		pose.q_gimbal_ned = matrix::Quatf(matrix::Eulerf(pose.gimbal_roll_ned, pose.gimbal_pitch_ned, pose.gimbal_yaw_ned));
	#else
		// 吊舱旋转顺序保持不变：俯仰(pitch，Y轴) → 滚转(roll，X轴) → 偏航(yaw，Z轴)
		matrix::Quatf q_pitch(matrix::Eulerf(0, _gimbal_pitch, 0));
		matrix::Quatf q_roll(matrix::Eulerf(_gimbal_roll, 0, 0));
		matrix::Quatf q_yaw(matrix::Eulerf(0, 0, _gimbal_yaw));
		matrix::Quatf q_gimbal_body = q_yaw * q_roll * q_pitch;
		pose.q_gimbal_ned = q_veh_ned * q_gimbal_body;

		matrix::Eulerf euler_gimbal_ned(pose.q_gimbal_ned);
		pose.gimbal_roll_ned = euler_gimbal_ned.phi();
		pose.gimbal_pitch_ned = euler_gimbal_ned.theta();
		pose.gimbal_yaw_ned = euler_gimbal_ned.psi();
	#endif

	pose.valid = true;
	return true;
}

bool AttackVision::build_guidance_command(
	const VehicleGuidanceState &veh,
	const GimbalNedPose &gimbal,
	GuidanceCommand &cmd)
{
	if (!veh.valid || !gimbal.valid) {
		return false;
	}

	const float att_kp = _param_av_att_kp.get();
	const float max_forward_v = _param_av_forward_v.get();
	const float max_att_error = 0.5f;
	const float max_vz = 0.8f;

	matrix::Vector3f forward_vec_ned = gimbal.q_gimbal_ned.rotateVector(matrix::Vector3f(1.f, 0.f, 0.f));
	if (forward_vec_ned.norm() > 1e-3f) {
		forward_vec_ned.normalize();
	}

	cmd.vx_ned = forward_vec_ned(0) * max_forward_v;
	cmd.vy_ned = forward_vec_ned(1) * max_forward_v;
	cmd.vz_ned = math::constrain(forward_vec_ned(2) * max_forward_v, -max_vz, max_vz);

	float roll_error = math::constrain(gimbal.gimbal_roll_ned - veh.veh_roll, -max_att_error, max_att_error);
	float pitch_error = math::constrain(gimbal.gimbal_pitch_ned - veh.veh_pitch, -max_att_error, max_att_error);
	float yaw_error = matrix::wrap_pi(gimbal.gimbal_yaw_ned - veh.veh_yaw);
	yaw_error = math::constrain(yaw_error, -max_att_error, max_att_error);

	cmd.target_roll = math::constrain(veh.veh_roll + att_kp * roll_error, -M_PI_4_F, M_PI_4_F);
	cmd.target_pitch = math::constrain(veh.veh_pitch + att_kp * pitch_error, -M_PI_2_F / 3.0f, M_PI_2_F / 3.0f);
	cmd.target_yaw = matrix::wrap_pi(veh.veh_yaw + att_kp * yaw_error);
	cmd.target_yaw_rate = 0.0f;
	cmd.valid = true;
	return true;
}

void AttackVision::publish_guidance_command(const GuidanceCommand &cmd)
{
	if (!cmd.valid) {
		return;
	}

	publish_attitude_velocity_control(
		cmd.target_roll,
		cmd.target_pitch,
		cmd.target_yaw,
		cmd.vx_ned,
		cmd.vy_ned,
		cmd.vz_ned,
		cmd.target_yaw_rate);
}

// ========== 新增：姿态+速度控制发布函数 ==========
/**
 * @brief 发布姿态+速度控制指令
 * @param target_roll 目标横滚角（弧度）
 * @param target_pitch 目标俯仰角（弧度）
 * @param target_yaw 目标偏航角（弧度）
 * @param vx_ned 北向速度（m/s）
 * @param vy_ned 东向速度（m/s）
 * @param target_yaw_rate 偏航角速度（rad/s）
 */
void AttackVision::publish_attitude_velocity_control(
	float target_roll, float target_pitch,
	float target_yaw, float vx_ned,
	float vy_ned, float vz_ned,
	float target_yaw_rate)
{
	PX4_DEBUG("guidance target r=%.2f p=%.2f y=%.2f vx=%.2f vy=%.2f vz=%.2f",
		(double)target_roll,
		(double)target_pitch,
		(double)target_yaw,
		(double)vx_ned,
		(double)vy_ned,
		(double)vz_ned);

	if (!PX4_ISFINITE(vx_ned) || !PX4_ISFINITE(vy_ned) || !PX4_ISFINITE(vz_ned) || !PX4_ISFINITE(target_yaw)) {
		PX4_WARN("skip invalid guidance setpoint");
		return;
	}

	offboard_control_mode_s ocm{};
	ocm.timestamp = hrt_absolute_time();
	ocm.position = false;
	ocm.velocity = true;
	ocm.acceleration = false;
	ocm.attitude = false;
	ocm.body_rate = false;
	ocm.thrust_and_torque = false;
	ocm.direct_actuator = false;
	_offboard_ctrl_pub.publish(ocm);

	trajectory_setpoint_s sp{};
	sp.timestamp = ocm.timestamp;
	sp.position[0] = NAN;
	sp.position[1] = NAN;
	sp.position[2] = NAN;
	sp.velocity[0] = vx_ned;
	sp.velocity[1] = vy_ned;
	sp.velocity[2] = vz_ned;
	sp.acceleration[0] = NAN;
	sp.acceleration[1] = NAN;
	sp.acceleration[2] = NAN;
	sp.yaw = target_yaw;
	sp.yawspeed = PX4_ISFINITE(target_yaw_rate) ? target_yaw_rate : 0.0f;
	_traj_sp_pub.publish(sp);
}

/**
 * @brief 打印无人机实时状态信息（位置、速度、加速度）
 */
void AttackVision::print_drone_status()
{
	vehicle_local_position_s local_pos{};
	if (_vehicle_local_position_sub.copy(&local_pos)) {
		// PX4_INFO("=== 无人机实时状态 ===");
		// PX4_INFO("位置: X=%.2fm, Y=%.2fm, Z=%.2fm",
		// 		(double)local_pos.x, (double)local_pos.y, (double)local_pos.z);
		// PX4_INFO("速度: Vx=%.2fm/s, Vy=%.2fm/s, Vz=%.2fm/s",
		// 		(double)local_pos.vx, (double)local_pos.vy, (double)local_pos.vz);
		// PX4_INFO("加速度: Ax=%.2fm/s², Ay=%.2fm/s², Az=%.2fm/s²",
		// 		(double)local_pos.ax, (double)local_pos.ay, (double)local_pos.az);
		// PX4_INFO("偏航角: %.2f°", (double)(local_pos.heading * 180.0f / M_PI_F));
	} else {
		PX4_WARN("无法获取无人机位置信息");
	}
}

/**
 * @brief 仿真模式下切换到Offboard模式（跳过RC检测）
 * @return true=切换成功，false=切换中或失败
 */
bool AttackVision::switch_to_offboard_sim()
{
	vehicle_status_s vs{};
	if (!_vehicle_status_sub.copy(&vs)) {
		PX4_WARN("无法获取vehicle_status");
		return false;
	}

	PX4_INFO("仿真模式：当前状态: nav_state=%d, arming_state=%d", vs.nav_state, vs.arming_state);

	// 检查是否已解锁
	if (vs.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		PX4_WARN("飞控未解锁，无法切换到Offboard模式");
		return false;
	}

	// 如果已经在Offboard模式
	if (vs.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
		PX4_INFO("已在Offboard模式");
		return true;
	}

	// 限制命令发布频率
	uint64_t now = hrt_absolute_time();
	if (now - _last_cmd_publish_time < MIN_CMD_INTERVAL_US) {
		return false;
	}

	PX4_INFO("仿真模式：发送切换到Offboard模式命令");

	// 发送模式切换命令
	vehicle_command_s cmd{};
	cmd.timestamp = now;
	cmd.param1 = (float)1;  // 主模式
	cmd.param2 = (float)6;  // PX4_CUSTOM_MAIN_MODE_OFFBOARD
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	cmd.target_system = 1;
	cmd.target_component = 1;
	cmd.source_system = 1;
	cmd.source_component = 1;
	cmd.confirmation = 0;
	cmd.from_external = false;

	_vehicle_cmd_pub.publish(cmd);
	_last_cmd_publish_time = now;

	PX4_INFO("已发送切换到Offboard模式命令（仿真模式）");
	return false;
}


// /**
//  * @brief 主运行循环
//  *
//  * 工作流程：
//  * 1. 检查模块使能开关
//  * 2. 打开串口
//  * 3. 循环读取串口数据并解析帧
//  * 4. 根据锁定状态和超时情况，切换模式并发布控制指令
//  */
void AttackVision::Run()
{
	PX4_INFO("=== Attack Vision Run() Started ===");
	// 检查模块使能开关

	if (_param_av_en.get() == 0) {
		PX4_INFO("Module disabled (AAATTKVIS_EN=0)");
		return; // 若未启用，直接退出循环
	}

	if (_param_av_en.get() <= 0) {
		PX4_WARN("AAATTKVIS_EN disabled");
		exit_and_cleanup();
		return;
	}

	PX4_INFO("Attack Vision module starting...");

	// 打开串口
	if (!open_uart()) {
		PX4_ERR("UART open failed");
		exit_and_cleanup();
		return;
	}

	// 确认串口状态
	if (_fd >= 0) {
		PX4_INFO("Attack Vision module started successfully - Using %s",
			#ifdef __PX4_POSIX
				"virtual serial port"
			#else
					"hardware UART /dev/ttyS5"
			#endif
				);
	} else {
		PX4_ERR("UART file descriptor invalid after open");
		exit_and_cleanup();
		return;
	}

	// 在仿真模式下，跳过RC检测，直接进入Offboard
	#ifdef __PX4_POSIX
		PX4_INFO("Simulation Mode: waiting for commander mode OFFBOARD before guidance");
	#endif


	#ifdef __PX4_POSIX
		if (!_virtual_gimbal_initialized) {
			_fixed_gimbal_roll_ned = 0.0f;
			_fixed_gimbal_pitch_ned = 0.0f;
			_fixed_gimbal_yaw_ned = M_PI_4_F + M_PI_2_F / 3.0f;

			PX4_INFO("【仿真模式】固定吊舱NED目标姿态初始化:");
			PX4_INFO("  - 目标横滚: %.2f° (%.6f rad)", 0.0, (double)_fixed_gimbal_roll_ned);
			PX4_INFO("  - 目标俯仰: %.2f° (%.6f rad)", 0.0, (double)_fixed_gimbal_pitch_ned);
			PX4_INFO("  - 目标偏航: %.2f° (%.6f rad)", (double)(_fixed_gimbal_yaw_ned * 180.0f / M_PI_F), (double)_fixed_gimbal_yaw_ned);
			PX4_INFO("注意：无人机将尝试跟踪这个固定的NED姿态目标");

			// 4. 为了兼容现有代码，仍需设置机体相对姿态（设为0，表示吊舱与机体对齐）
			// 因为后面计算误差时，我们直接使用固定NED姿态与无人机NED姿态的差值
			_gimbal_roll = 0.0f;
			_gimbal_pitch = 0.0f;
			_gimbal_yaw = 0.0f;

			_virtual_gimbal_initialized = true;
		}
	#endif

	const uint64_t frame_timeout_us = 200000;
	const uint64_t control_timeout_us = 50000;
	static uint64_t last_status_time = 0;
	static uint64_t last_control_time = 0;
	static uint64_t last_drone_status_time = 0;

	while (!should_exit()) {
		const uint64_t now = hrt_absolute_time();

		// 1) 更新 RC 模式。仿真保留原差异：不依赖实体遥控器，直接视为 Offboard 授权档。
		#ifdef __PX4_POSIX
			_current_rc_mode = RCMode::MODE_OFFBOARD;
		#else
			const RCMode new_rc_mode = parse_rc_mode();
			if (new_rc_mode != _current_rc_mode) {
				PX4_INFO("RC mode changed: %d -> %d", (int)_current_rc_mode, (int)new_rc_mode);
				_current_rc_mode = new_rc_mode;
			}
		#endif

		const bool rc_offboard = (_current_rc_mode == RCMode::MODE_OFFBOARD);

		// 2) 更新上位机协同状态。
		if (_external_mission_active_sub.updated()) {
			external_mission_active_s external_mission{};
			if (_external_mission_active_sub.copy(&external_mission)) {
				_external_mission_active = external_mission.external_mission_active;
			}
		}

		// 3) 更新吊舱帧。
		static uint64_t last_frame_log_time = 0;
		if (try_read_frame() && now - last_frame_log_time > 3000000) {
			PX4_INFO("成功解析帧: lock=%d, pix=(%d,%d)",
				(int)_lock_active, (int)_pix_offset_x, (int)_pix_offset_y);
			last_frame_log_time = now;
		}

		// 4) 更新飞控状态和统一接管判定。
		vehicle_status_s vehicle_status{};
		const bool has_vehicle_status = _vehicle_status_sub.copy(&vehicle_status);
		const bool frame_valid_recent = (_last_frame_time_us > 0) && ((now - _last_frame_time_us) < frame_timeout_us);
		const bool vehicle_armed = has_vehicle_status && (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
		const bool vehicle_in_offboard = has_vehicle_status && (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

		_allow_takeover = !_guidance_paused && rc_offboard && !_external_mission_active && _lock_active;
		const bool can_control = _allow_takeover && frame_valid_recent && vehicle_armed;

		if (_guidance_paused) {
			publish_position_offboard_heartbeat();
		}

		attack_vision_status_s status{};
		status.lock_active = _lock_active;
		status.rc_offboard = rc_offboard;
		status.external_mission_active = _external_mission_active;
		status.allow_takeover = _allow_takeover;
		status.pix_offset_x = _pix_offset_x;
		status.pix_offset_y = _pix_offset_y;
		status.timestamp = now;
		_attack_vision_status_pub.publish(status);

		// 5) 最高优先级：上位机 active 时完全让路，不主动切 HOLD。
		if (_external_mission_active) {
			if (_module_state != ModuleState::HOLD) {
				PX4_INFO("External mission active -> attack_vision idle");
				_module_state = ModuleState::HOLD;
				_switch_start_time = 0;
			}
			usleep(20000);
			continue;
		}

		// 6) 遥控器退出末制导授权：实机切回 RC 对应模式，未知档位悬停；仿真不会进入该分支。
		if (!rc_offboard) {
			if (_module_state != ModuleState::HOLD) {
				PX4_INFO("RC not in Offboard -> stop guidance");
				if (_current_rc_mode == RCMode::MODE_UNKNOWN) {
					switch_to_hold();
				} else {
					switch_to_rc_mode(_current_rc_mode);
				}
				_module_state = ModuleState::HOLD;
				_switch_start_time = 0;
			}
			usleep(20000);
			continue;
		}

		// 7) 显式状态机：HOLD 等待接管，SWITCHING 预热并切 Offboard，OFFBOARD 执行末制导。
		switch (_module_state) {
		case ModuleState::HOLD:
			if (can_control) {
				if (vehicle_in_offboard) {
					PX4_INFO("已在Offboard模式 - 开始末制导%s",
					#ifdef __PX4_POSIX
						"(仿真模式)"
					#else
						"(RC授权)"
					#endif
						);
					_module_state = ModuleState::OFFBOARD;
				} else {
					PX4_INFO("末制导条件满足，开始切换到Offboard模式");
					_module_state = ModuleState::SWITCHING_TO_OFFBOARD;
					_switch_start_time = now;
				}
			}
			break;

		case ModuleState::SWITCHING_TO_OFFBOARD:
			if (!can_control) {
				PX4_INFO("Offboard切换条件丢失 -> 悬停");
				if (vehicle_armed) {
					switch_to_hold();
				}
				_module_state = ModuleState::HOLD;
				_switch_start_time = 0;
				break;
			}

			if (vehicle_in_offboard) {
				PX4_INFO("Offboard切换完成 - 开始末制导");
				_module_state = ModuleState::OFFBOARD;
				break;
			}

			publish_offboard_velocity(0.0f, 0.0f, 0.0f, 0.0f);
			last_control_time = now;

			#ifdef __PX4_POSIX
				if (switch_to_offboard_sim()) {
					_module_state = ModuleState::OFFBOARD;
				}
			#else
				switch_to_offboard();
			#endif
			break;

		case ModuleState::OFFBOARD:
			if (!can_control) {
				PX4_INFO("末制导条件丢失 -> 悬停");
				if (vehicle_armed) {
					switch_to_hold();
				}
				_module_state = ModuleState::HOLD;
				_switch_start_time = 0;
				break;
			}

			if (!vehicle_in_offboard) {
				PX4_WARN("飞控已退出Offboard，重新进入切换状态");
				_module_state = ModuleState::SWITCHING_TO_OFFBOARD;
				_switch_start_time = now;
				break;
			}

			if ((now - last_control_time) > control_timeout_us) {
				handle_guidance();
				last_control_time = now;
			}
			break;
		}

		if (now - last_status_time > 5000000) {
			const uint64_t time_since_last = (_last_frame_time_us > 0) ? (now - _last_frame_time_us) : UINT64_MAX;
			if (has_vehicle_status) {
				PX4_INFO("飞控状态: nav_state=%d, arming_state=%d, rc_mode=%d, external_active=%d, lock=%d, frame_valid=%d, allow_takeover=%d, can_control=%d, module_state=%d",
					vehicle_status.nav_state,
					vehicle_status.arming_state,
					(int)_current_rc_mode,
					(int)_external_mission_active,
					(int)_lock_active,
					(int)frame_valid_recent,
					(int)_allow_takeover,
					(int)can_control,
					(int)_module_state);
			}

			PX4_INFO("Target Lock: %s, Pixel Offset: X=%d, Y=%d",
					_lock_active ? "YES" : "NO",
					(int)_pix_offset_x,
					(int)_pix_offset_y);

			if (time_since_last > 1000000) {
				PX4_WARN("长时间未收到帧数据: %.1f 秒", (double)(time_since_last) / 1000000.0);
			}

			last_status_time = now;
		}

		if (now - last_drone_status_time > 1000000) {
			print_drone_status();
			last_drone_status_time = now;
		}

		usleep(20000);
	}

	safe_stop_guidance();
	close_uart();
	exit_and_cleanup();
}


extern "C" __EXPORT int attack_vision_main(int argc, char *argv[])
{
	if (argc > 1 && strcmp(argv[1], "stop") == 0) {
		return AttackVision::soft_stop_command();
	}

	if (argc > 1 && strcmp(argv[1], "kill") == 0) {
		char *stop_argv[] = {argv[0], const_cast<char *>("stop")};
		return AttackVision::main(2, stop_argv);
	}

	return AttackVision::main(argc, argv);
}
