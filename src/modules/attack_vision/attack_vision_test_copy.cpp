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
AttackVision::AttackVision()
	: ModuleParams(nullptr)
	, WorkItem(MODULE_NAME, px4::wq_configurations::ttyS5)
{
    	// 在SITL模式下使用虚拟串口
	PX4_INFO("Using virtual serial port for SITL simulation");
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
	AttackVision *instance = new AttackVision();
	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;
		instance->ScheduleNow();
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

		吊舱串口连接到飞控TELEM2口（/dev/ttyS2）

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

	PX4_INFO("Attack Vision Status:");
	PX4_INFO("  UART: %s", (_fd >= 0) ? "OPEN" : "CLOSED");
	PX4_INFO("  Target Lock: %s", _lock_active ? "YES" : "NO");
	PX4_INFO("  Pixel Offset: X=%d, Y=%d", (int)_pix_offset_x, (int)_pix_offset_y);
	PX4_INFO("  Module State: %d", (int)_module_state);
	PX4_INFO("  Last Frame: %.3f ms ago", (double)(time_since_last_frame) / 1000.0);

	// 如果长时间没有收到帧，输出警告
	if (time_since_last_frame > 500000) {  // 500ms
		PX4_WARN("  No frame received for %.3f ms - check virtual serial", (double)(time_since_last_frame) / 1000.0);
	}

	// 在print_status函数中添加
	const char* state_str = "UNKNOWN";
	switch (_module_state) {
	case ModuleState::HOLD: state_str = "HOLD"; break;
	case ModuleState::SWITCHING_TO_OFFBOARD: state_str = "SWITCHING_TO_OFFBOARD"; break;
	case ModuleState::OFFBOARD: state_str = "OFFBOARD"; break;
	}
	PX4_INFO("  Module State: %s", state_str);

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
	_fd = ::open(dev, O_RDWR | O_NOCTTY);
	if (_fd < 0) {
		PX4_ERR("open %s failed", dev);
		return false;
	}
	PX4_INFO("UART opened: %s, fd=%d", dev, _fd);
	return configure_uart(_param_av_baud.get());
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
		static int success_count = 0;
		if (success_count < 10) {
			PX4_INFO("成功解析虚拟串口帧: lock=%d, pix=(%d,%d)",
				(int)_lock_active, (int)_pix_offset_x, (int)_pix_offset_y);
			success_count++;
		}
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
	// ========== 解析关键字段 ==========
	// 第5-6字节：吊舱状态（UINT16，小端序）
	uint16_t status_5_6 = (uint16_t)_buf[4] | ((uint16_t)_buf[5] << 8);
	// 第9字节：伺服状态
	uint8_t servo_state = _buf[8];

	// 调试信息：打印原始数据
	static int debug_count = 0;
	if (debug_count < 5) {
		PX4_INFO("原始数据 - status_5_6: 0x%04X, servo_state: 0x%02X",
			status_5_6, servo_state);
		debug_count++;
	}

	// 锁定状态判断：检查第9位（从0开始计数）
	// 在vserial.cpp中，锁定状态设置在status1的第9位
	bool locking = (status_5_6 & (1 << 9)) != 0;

	// 锁定有效条件：锁定标识有效 AND 伺服状态为跟踪模式（0x07）
	_lock_active = locking && (servo_state == 0x07);

	// 第59-60字节：目标脱靶量-方位方向（INT16，小端序，单位：像素）
	_pix_offset_x = (int16_t)((uint16_t)_buf[58] | ((uint16_t)_buf[59] << 8));
	// 第61-62字节：目标脱靶量-俯仰方向（INT16，小端序，单位：像素）
	_pix_offset_y = (int16_t)((uint16_t)_buf[60] | ((uint16_t)_buf[61] << 8));

	// 调试信息
	if (debug_count < 10) {
		// PX4_INFO("解析结果 - 锁定=%d, 脱靶量=(%d,%d), locking=%d, servo=0x%02X",
		// 	(int)_lock_active, (int)_pix_offset_x, (int)_pix_offset_y,
		// 	(int)locking, servo_state);
	}
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

	// 使用标准的MAVLink模式切换命令
	vehicle_command_s cmd{};
	cmd.timestamp = now;
	cmd.param1 = 1.0f;  // 主模式
	cmd.param2 = 6.0f;  // PX4_CUSTOM_MAIN_MODE_OFFBOARD
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
	cmd.param2 = (float)5;      // PX4_CUSTOM_MAIN_MODE_AUTO
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

	// 设置速度控制（NED坐标系：velocity[0]=North, velocity[1]=East, velocity[2]=Down）
	sp.velocity[0] = vx;  // North方向速度（前向）
	sp.velocity[1] = vy;  // East方向速度（右侧）
	sp.velocity[2] = -vz; // Down方向速度（向下为正，所以取负）
	sp.yawspeed = yaw_rate;  // 偏航角速度

	_traj_sp_pub.publish(sp);
}

/**
 * @brief 处理制导逻辑
 *
 * 根据目标脱靶量（像素偏差）计算速度控制指令：
 * - 像素偏差转换为速度指令：v = -KP * pixel_offset
 * - 负号表示：目标在图像右侧（正像素），飞控向左移动（负速度）
 * - 应用死区：小于死区阈值的偏差不产生控制
 * - 速度限幅：限制最大速度，确保安全
 */
void AttackVision::handle_guidance()
{
	const float kp = _param_av_kp.get();      // 速度控制增益（m/s每像素）
	const float dead = _param_av_dead.get();  // 像素死区
	const float maxv = _param_av_max_v.get(); // 最大速度限制

	float ex = (float)_pix_offset_x;  // 方位方向像素偏差
	float ey = (float)_pix_offset_y;  // 俯仰方向像素偏差

	if (PX4_ISFINITE(ex) && PX4_ISFINITE(ey)) {
		// 应用死区：小于死区阈值时清零
		if (fabsf(ex) < dead) ex = 0.f;
		if (fabsf(ey) < dead) ey = 0.f;

		// 计算速度指令：v = -KP * pixel（负号保证控制方向正确）
		// 限幅保护：限制在[-maxv, maxv]范围内
		float vx = -math::constrain(kp * ex, -maxv, maxv);  // 前向速度（对应方位偏差）
		float vy = -math::constrain(kp * ey, -maxv, maxv);  // 横向速度（对应俯仰偏差）

		// 发布速度控制指令（垂直速度和偏航角速度保持为0）
		publish_offboard_velocity(vx, vy, 0.f, 0.f);
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
	// 检查模块使能开关
	if (_param_av_en.get() <= 0) {
		PX4_WARN("AAATTKVIS_EN disabled");
		exit_and_cleanup();
		return;
	}

	// 打开串口
	if (!open_uart()) {
		PX4_ERR("UART open failed");
		exit_and_cleanup();
		return;
	}

	PX4_INFO("攻击视觉模块启动成功 - 使用虚拟串口");

	const uint64_t frame_timeout_us = 200000;  // 200ms超时
	static uint64_t last_status_time = 0;
	static int frame_count = 0;

	while (!should_exit()) {
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

		if (_lock_active && frame_valid_recent && has_vehicle_status) {
			if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
				// 飞控已解锁
				if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
				// 已在Offboard模式，执行制导
				if (_module_state != ModuleState::OFFBOARD) {
					PX4_INFO("已进入Offboard模式 - 开始制导");
					_module_state = ModuleState::OFFBOARD;
				}
				handle_guidance();
				} else {
				// 不在Offboard模式，尝试切换
				if (_module_state != ModuleState::SWITCHING_TO_OFFBOARD) {
					PX4_INFO("尝试切换到Offboard模式");
					_module_state = ModuleState::SWITCHING_TO_OFFBOARD;

					// 关键：在切换模式前先发布一次控制信号
					PX4_INFO("先发布零速度控制信号以满足PX4要求");
					publish_offboard_velocity(0.0f, 0.0f, 0.0f, 0.0f);
				}

				// 短暂延迟后发送模式切换命令
				_switch_start_time = hrt_absolute_time();

				// 等待100ms确保控制信号已被接收
				if (hrt_absolute_time() - _switch_start_time > 100000) {
					switch_to_offboard();
				}
				}
			} else {
				PX4_WARN("目标已锁定但飞控未解锁，无法进入Offboard模式");
				if (_module_state != ModuleState::HOLD) {
				_module_state = ModuleState::HOLD;
				}
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
		uint64_t now = hrt_absolute_time();
		if (now - last_status_time > 5000000) {
		uint64_t time_since_last = now - _last_frame_time_us;
		PX4_INFO("状态: 模块状态=%d, 锁定=%d, 脱靶量=(%d,%d), 最后帧 %.1f 秒前",
			(int)_module_state, (int)_lock_active, (int)_pix_offset_x, (int)_pix_offset_y,
			(double)(time_since_last) / 1000000.0);

		if (has_vehicle_status) {
			PX4_INFO("飞控状态: 导航状态=%d, 解锁状态=%d",
			vehicle_status.nav_state, vehicle_status.arming_state);
		}

		if (time_since_last > 1000000) {
			PX4_WARN("长时间未收到帧数据: %.1f 秒", (double)(time_since_last) / 1000000.0);
		}
		last_status_time = now;
		}

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


