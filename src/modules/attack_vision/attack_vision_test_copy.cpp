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
	return print_usage("unknown command");
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
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

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

		// 调试信息
		// static int success_count = 0;
		// if (success_count < 10) {
		// 	PX4_INFO("成功解析虚拟串口帧: lock=%d, pix=(%d,%d)",
		// 		(int)_lock_active, (int)_pix_offset_x, (int)_pix_offset_y);
		// 	success_count++;
		// }
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
	// 硬件模式：原有的文件描述符读取逻辑
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
		_buf[_buf_len++] = byte;

		// 检查是否收集到完整帧
		if (_buf_len >= FRAME_LEN) {
		// 校验帧格式
		bool ok = validate_frame(_buf);
		if (ok) {
			_last_frame_time_us = hrt_absolute_time();
			parse_frame_data();
			_buf_len = 0;
			return true;
		} else {
			// 校验失败，滑动窗口
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
	if (debug_frame_count < 10) {
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

	// 调试信息：打印原始数据
	// PX4_INFO("原始锁定数据-status_5_6:");
	// PX4_INFO("0x%04X(字节[4]=0x%02X,字节[5]=0x%02X), servo_state: 0x%02X",
	// 	status_5_6, _buf[4], _buf[5], servo_state);

	// 详细解析状态字节的每一位
	// PX4_INFO("状态字节分析:");
	// for (int bit = 0; bit < 16; bit++) {
	// 	if (status_5_6 & (1 << bit)) {
	// 	PX4_INFO("  Bit%d: 1", bit);
	// 	}
	// }

	// 特别注意Bit9和Bit10（协议中的锁定状态位）
	bool bit9 = (status_5_6 & (1 << 9)) != 0;
	bool bit10 = (status_5_6 & (1 << 10)) != 0;
	// PX4_INFO("锁定状态位: Bit9=%d, Bit10=%d", (int)bit9, (int)bit10);

	// 根据协议第14页，Bit9~Bit10表示目标锁定标识位：
	// 00: 默认（无效）
	// 01: 锁定中
	// 10: 锁定预测
	// 11: 退出锁定
	int lock_state = ((bit10 ? 1 : 0) << 1) | (bit9 ? 1 : 0);
	const char* lock_state_str = "";
	switch (lock_state) {
		case 0: lock_state_str = "00-默认(无效)"; break;
		case 1: lock_state_str = "01-锁定中"; break;
		case 2: lock_state_str = "10-锁定预测"; break;
		case 3: lock_state_str = "11-退出锁定"; break;
	}
	PX4_INFO("锁定标识: %s", lock_state_str);

	// 锁定有效条件：锁定标识位为01（锁定中）且伺服状态为跟踪模式（0x07）
	// bool locking = (lock_state == 1);  // 01状态表示锁定中
	bool locking = (lock_state == 1) || (lock_state == 2);  // 01(锁定中) 或 10(锁定预测)

	// 调试输出伺服状态
	const char* servo_state_str = "";
	switch (servo_state) {
		case 0x01: servo_state_str = "载荷关"; break;
		case 0x02: servo_state_str = "手动"; break;
		case 0x03: servo_state_str = "收藏"; break;
		case 0x04: servo_state_str = "数引"; break;
		case 0x05: servo_state_str = "航向锁定"; break;
		case 0x06: servo_state_str = "扫描"; break;
		case 0x07: servo_state_str = "跟踪"; break;
		case 0x08: servo_state_str = "垂直下视"; break;
		case 0x09: servo_state_str = "陀螺自动较漂"; break;
		case 0x0A: servo_state_str = "陀螺温度较漂"; break;
		case 0x0B: servo_state_str = "航向随动"; break;
		case 0x0C: servo_state_str = "归中"; break;
		case 0x0D: servo_state_str = "手动陀螺较漂"; break;
		case 0x0E: servo_state_str = "姿态指引"; break;
		default: servo_state_str = "未知"; break;
	}
	PX4_INFO("伺服状态: 0x%02X (%s)", servo_state, servo_state_str);

	_lock_active = locking && (servo_state == 0x07);

	// 第59-60字节：目标脱靶量-方位方向（INT16，小端序，单位：像素）
	_pix_offset_x = (int16_t)((uint16_t)_buf[58] | ((uint16_t)_buf[59] << 8));
	// 第61-62字节：目标脱靶量-俯仰方向（INT16，小端序，单位：像素）
	_pix_offset_y = (int16_t)((uint16_t)_buf[60] | ((uint16_t)_buf[61] << 8));

	// 调试输出脱靶量
	PX4_INFO("[58]=0x%02X, [59]=0x%02X, [60]=0x%02X, [61]=0x%02X",
		_buf[58], _buf[59], _buf[60], _buf[61]);

	// PX4_INFO("解析结果 - 锁定=%d, 脱靶量=(%d,%d), locking=%d, servo=0x%02X(%s)",
	// 	(int)_lock_active, (int)_pix_offset_x, (int)_pix_offset_y,
	// 	(int)locking, servo_state, servo_state_str);
	PX4_INFO("=========================================");
}



/**
 * @brief 切换到Offboard模式
 * @return true=已在Offboard模式且已解锁，false=正在切换中或未解锁
 *
 * 注意：需要飞控已解锁且允许Offboard模式（通过地面站参数设置）
 */
bool AttackVision::switch_to_offboard()
{
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

	// 关键修复：先持续发布控制信号，然后再发送模式切换命令
	PX4_INFO("准备切换到Offboard模式，先发布控制信号...");

	// 发布零速度控制信号
	// publish_offboard_velocity(0.0f, 0.0f, 0.0f, 0.0f);

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
	cmd.param1 = (float)1;      // 主模式
	cmd.param2 = (float)4;      // PX4_CUSTOM_MAIN_MODE_AUTO
	cmd.param3 = (float)3;      // PX4_CUSTOM_SUB_MODE_AUTO_LOITER
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

/**
 * @brief 发布Offboard速度控制指令
 * @param vx 前向速度（m/s，机体系）
 * @param vy 横向速度（m/s，机体系）
 * @param vz 垂直速度（m/s，向上为正）
 * @param yaw_rate 偏航角速度（rad/s）
 *
 * 需要同时发布offboard_control_mode和trajectory_setpoint两个话题
 */
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
	sp.velocity[2] = vz; // 关键：保持高度，垂直速度设为0

	_traj_sp_pub.publish(sp);

	PX4_DEBUG("发布速度控制: vx=%.2f, vy=%.2f, vz=%.2f", (double)vx, (double)vy, (double)0.0f);

	#ifdef __PX4_POSIX
	VSerial::get_instance().set_control_input(vx, vy, vz);  // 传入三个速度分量

	// 调试：获取当前脱靶量状态
	int16_t current_offset_x, current_offset_y;
	VSerial::get_instance().get_target_offset(current_offset_x, current_offset_y);

	// static int debug_count = 0;
	// if (debug_count < 20) {
	// 	PX4_INFO("闭环控制: 发送控制量(%.3f,%.3f,%.3f), 当前脱靶量(%d,%d)",
	// 			(double)vx, (double)vy, (double)vz, current_offset_x, current_offset_y);
	// 	debug_count++;
	// }
	#endif

}


/**
 * @brief 处理制导逻辑
 *
 * 新的控制策略：
 * - 方向偏差（pix_offset_x）控制横向速度（vy）
 * - 俯仰偏差（pix_offset_y）控制垂直速度（vz）
 * - 前向保持恒定速度（vx）
 * - 偏航角速度保持为0（保持当前航向）
 */
void AttackVision::handle_guidance()
{
	const float kp = _param_av_kp.get();      // 速度控制增益（m/s每像素）
	const float dead = _param_av_dead.get();  // 像素死区
	const float maxv = _param_av_max_v.get(); // 最大速度限制

	float ex = (float)_pix_offset_x;  // 方位方向像素偏差 -> 控制横向
	float ey = (float)_pix_offset_y;  // 俯仰方向像素偏差 -> 控制垂直

	if (PX4_ISFINITE(ex) && PX4_ISFINITE(ey)) {
		// 应用死区：小于死区阈值时清零
		if (fabsf(ex) < dead) ex = 0.f;
		if (fabsf(ey) < dead) ey = 0.f;

		// 计算机体系速度（你的逻辑）
		float vx_body = _forward_velocity;  // 机头方向
		float vy_body = -math::constrain(kp * ex, -maxv, maxv);  // 右侧方向
		float vz_body = -math::constrain(kp * ey, -maxv, maxv);  // 向下方向

		// 获取当前偏航角（航向）
		vehicle_local_position_s local_pos{};
		if (_vehicle_local_position_sub.copy(&local_pos)) {
		float yaw = local_pos.heading; // 当前偏航角（弧度）

		// 将机体系速度转换为NED坐标系速度
		// v_north = vx_body * cos(yaw) - vy_body * sin(yaw)
		// v_east  = vx_body * sin(yaw) + vy_body * cos(yaw)
		float v_north = vx_body * cosf(yaw) - vy_body * sinf(yaw);
		float v_east  = vx_body * sinf(yaw) + vy_body * cosf(yaw);

		// 第671-675行修改为：
		PX4_INFO("机体系->NED转换: 偏航=%.1f°, 机体系(%.2f,%.2f,%.2f) -> NED(%.2f,%.2f,%.2f)",
		(double)(yaw * 180.0f / M_PI_F),
		(double)vx_body, (double)vy_body, (double)vz_body,
		(double)v_north, (double)v_east, (double)vz_body);

		// 发布NED坐标系速度
		publish_offboard_velocity(v_north, v_east, vz_body, 0.0f);
		} else {
		PX4_WARN("无法获取偏航角，使用默认北向");
		publish_offboard_velocity(vx_body, 0.0f, 0.0f, 0.0f);
		}
	}
}

/**
 * @brief 打印无人机实时状态信息（位置、速度、加速度）
 */
void AttackVision::print_drone_status()
{
	vehicle_local_position_s local_pos{};
	if (_vehicle_local_position_sub.copy(&local_pos)) {
		PX4_INFO("=== 无人机实时状态 ===");
		PX4_INFO("位置: X=%.2fm, Y=%.2fm, Z=%.2fm",
				(double)local_pos.x, (double)local_pos.y, (double)local_pos.z);
		PX4_INFO("速度: Vx=%.2fm/s, Vy=%.2fm/s, Vz=%.2fm/s",
				(double)local_pos.vx, (double)local_pos.vy, (double)local_pos.vz);
		PX4_INFO("加速度: Ax=%.2fm/s², Ay=%.2fm/s², Az=%.2fm/s²",
				(double)local_pos.ax, (double)local_pos.ay, (double)local_pos.az);
		PX4_INFO("偏航角: %.2f°", (double)(local_pos.heading * 180.0f / M_PI_F));
	} else {
		PX4_WARN("无法获取无人机位置信息");
	}
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

	const uint64_t frame_timeout_us = 200000;  // 200ms超时
	const uint64_t control_timeout_us = 50000; // 50ms控制信号超时（20Hz）
	static uint64_t last_status_time = 0;
	static uint64_t last_control_time = 0;
	static uint64_t last_drone_status_time = 0; // 无人机状态打印计时器
	static int frame_count = 0;

	while (!should_exit()) {
		// 尝试读取并处理帧数据
		if (try_read_frame()) {
		// 成功读取到一帧数据
		if (frame_count < 10) {
			PX4_INFO("成功解析帧 %d: lock=%d, pix=(%d,%d)",
			frame_count, (int)_lock_active, (int)_pix_offset_x, (int)_pix_offset_y);
			frame_count++;
		}
		}

		// 获取vehicle_status
		vehicle_status_s vehicle_status{};
		bool has_vehicle_status = _vehicle_status_sub.copy(&vehicle_status);

		// 制导逻辑
		bool frame_valid_recent = (hrt_absolute_time() - _last_frame_time_us) < frame_timeout_us;

		// 关键修改：在Offboard模式下必须持续发布控制信号
		uint64_t now = hrt_absolute_time();
		bool need_control_publish = (now - last_control_time > control_timeout_us);

		if (_lock_active && frame_valid_recent && has_vehicle_status) {
			if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
				// 飞控已解锁
				if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
				// 已在Offboard模式，执行制导
					if (_module_state != ModuleState::OFFBOARD) {
						PX4_INFO("已进入Offboard模式 - 开始制导");
						_module_state = ModuleState::OFFBOARD;
					}

					// 关键：在Offboard模式下必须持续发布控制信号
					if (need_control_publish) {
						handle_guidance();
						last_control_time = now;
					}
				} else {
				// 不在Offboard模式，尝试切换
					if (_module_state != ModuleState::SWITCHING_TO_OFFBOARD) {
						PX4_INFO("尝试切换到Offboard模式");
						_module_state = ModuleState::SWITCHING_TO_OFFBOARD;

						_switch_start_time = now;
						// 关键：在切换模式前先发布控制信号
						PX4_INFO("先发布零速度控制信号以满足PX4要求");
						publish_offboard_velocity(0.0f, 0.0f, 0.0f, 0.0f);
						last_control_time = now;
					}
						switch_to_offboard();
				}
			} else {
				PX4_WARN("目标已锁定但飞控未解锁，无法进入Offboard模式");
				if (_module_state != ModuleState::HOLD) {
				_module_state = ModuleState::HOLD;
				}
				// 重置切换计时器
				_switch_start_time = 0;
			}
		} else {
		// 失锁或数据超时
			if (_module_state != ModuleState::HOLD) {
				PX4_INFO("条件不满足，切换回悬停模式");
				switch_to_hold();
				_module_state = ModuleState::HOLD;
			}
			// 重置切换计时器
			_switch_start_time = 0;
		}

		// 状态输出（每5秒）
		if (now - last_status_time > 5000000) {
		uint64_t time_since_last = now - _last_frame_time_us;
		// PX4_INFO("状态: 模块状态=%d, 锁定=%d, 脱靶量=(%d,%d), 最后帧 %.1f 秒前",
		// 	(int)_module_state, (int)_lock_active, (int)_pix_offset_x, (int)_pix_offset_y,
		// 	(double)(time_since_last) / 1000000.0);

		if (has_vehicle_status) {
			PX4_INFO("飞控状态: 导航状态=%d, 解锁状态=%d",
			vehicle_status.nav_state, vehicle_status.arming_state);
		}

		if (time_since_last > 1000000) {
			PX4_WARN("长时间未收到帧数据: %.1f 秒", (double)(time_since_last) / 1000000.0);
		}
		last_status_time = now;
		}

		if (now - last_drone_status_time > 1000000) {
			print_drone_status();
			last_drone_status_time = now;
		}

		// // 在handle_guidance调用后添加
		// PX4_INFO("控制指令 - 机体系: (%.2f, %.2f, %.2f) -> NED: (%.2f, %.2f, %.2f)",
		// 	(double)vx_body, (double)vy_body, (double)vz_body,
		// 	(double)v_north, (double)v_east, (double)vz_body);

		// 控制循环频率
		usleep(20000); // 50Hz
	}

	close_uart();
	exit_and_cleanup();
}



extern "C" __EXPORT int attack_vision_main(int argc, char *argv[])
{
	return AttackVision::main(argc, argv);
}


