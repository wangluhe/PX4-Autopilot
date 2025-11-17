#include "VisionGuidance.hpp"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#include <fcntl.h>
#include <unistd.h>
#include <poll.h>

// 在构造函数中初始化参数
#include "VisionGuidance.hpp"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include "vserial.h"

// 在构造函数中
VisionGuidance::VisionGuidance() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::ttyS5)
	{
	// 在SITL模式下使用虚拟串口
	_uart_port = "/dev/ttyV0"; // 虚拟串口设备名
	PX4_INFO("Using virtual serial port for SITL simulation");

	_baudrate = _param_baudrate.get();
	updateParams();
}

VisionGuidance::~VisionGuidance()
{
	close_uart();
}

// 添加缺失的 start() 方法实现
int VisionGuidance::start()
{
	_is_running = true;
	PX4_INFO("Vision Guidance module started");
	return 0;
}

int VisionGuidance::open_uart()
{
	// 初始化虚拟串口
	if (VSerial::get_instance().init() != 0) {
		PX4_ERR("failed to initialize virtual serial port");
		return -1;
	}
	_uart_fd = 1; // 虚拟文件描述符
	PX4_INFO("Virtual UART port opened successfully");
	return 0;
}

int VisionGuidance::read_uart_data(uint8_t *buffer, size_t length)
{
	if (_uart_fd < 0) {
		return -1;
	}

	// 从虚拟串口读取数据
	return VSerial::get_instance().read(buffer, length);
}

// setup_uart 函数在虚拟串口模式下不需要做任何事
int VisionGuidance::setup_uart()
{
    // 虚拟串口不需要设置
	return 0;
}

void VisionGuidance::close_uart()
{
    // 虚拟串口不需要关闭操作
	_uart_fd = -1;
}

// 添加缺失的 emergency_stop() 方法实现
void VisionGuidance::emergency_stop()
{
	PX4_WARN("EMERGENCY STOP: Deactivating guidance and stopping vehicle");
	_guidance_active = false;
	_current_velocity = matrix::Vector2f(0.0f, 0.0f);
	_position_setpoint_valid = false;
	set_guidance_state(GuidanceState::STATE_ERROR);

	// 发布停止指令
	position_setpoint_triplet_s pos_sp_triplet{};
	pos_sp_triplet.timestamp = hrt_absolute_time();
	pos_sp_triplet.current.timestamp = hrt_absolute_time();
	pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	pos_sp_triplet.current.valid = false;
	_pos_sp_triplet_pub.publish(pos_sp_triplet);
}

bool VisionGuidance::validate_frame(const VisionPodData &data)
{
	// Check headers and footer
	if (data.header1 != 0xFC || data.header2 != 0x2C || data.footer != 0xF0) {
		return false;
	}

	// Check checksum
	if (data.checksum != calculate_checksum(data)) {
		return false;
	}

	return true;
}

uint8_t VisionGuidance::calculate_checksum(const VisionPodData &data)
{
	const uint8_t *data_ptr = reinterpret_cast<const uint8_t*>(&data);
	uint8_t checksum = 0;

	// XOR bytes 3 to 62 (0-indexed, so indices 2 to 61)
	for (int i = 2; i < 62; i++) {
		checksum ^= data_ptr[i];
	}

	return checksum;
}

bool VisionGuidance::is_target_locked(const VisionPodData &data)
{
	// Extract target lock status from status1 (bits 9-10)
	uint16_t lock_status = (data.status1 >> 9) & 0x03;
	bool locked = (lock_status == 0x01 || lock_status == 0x02); // 01:锁定中, 10:锁定预测

	// PX4_INFO("Lock status: 0x%02X, Locked: %s", lock_status, locked ? "YES" : "NO");
	// PX4_INFO("Status1: 0x%04X", data.status1);

	return locked;
}

void VisionGuidance::process_pod_data(const VisionPodData &data)
{
	_last_valid_data = hrt_absolute_time();
	_last_pod_data = data;

	// Check if target is locked and guidance should be activated
	bool target_locked = is_target_locked(data);

	// PX4_INFO("Target locked: %s, Guidance active: %s",
	// 	target_locked ? "YES" : "NO",
	// 	_guidance_active ? "YES" : "NO");

	if (target_locked && !_guidance_active) {
		PX4_INFO("Attempting to activate guidance...");
		if (safety_checks_passed()) {
		activate_guidance();
		} else {
		PX4_WARN("Safety checks failed when trying to activate guidance");
		}
	} else if (!target_locked && _guidance_active) {
		PX4_INFO("Deactivating guidance due to target lost");
		deactivate_guidance();
	}

	// If guidance is active, calculate control commands
	if (_guidance_active) {
		// PX4_INFO("Guidance active, calculating commands");
		calculate_guidance_commands(data.off_target_azimuth, data.off_target_elevation);
	}

	update_guidance_status();
}

// 在 calculate_guidance_commands 函数中更新参数使用：
void VisionGuidance::calculate_guidance_commands(int16_t off_azimuth, int16_t off_elevation)
{
	// Convert integer parameters to float (divide by scaling factor)
	PX4_INFO("Offsets: az=%d, el=%d", off_azimuth, off_elevation);
	float kp_azimuth = _param_kp_azimuth.get() / 10000.0f;
	float kp_elevation = _param_kp_elevation.get() / 10000.0f;
	float max_velocity = _param_max_velocity.get() / 100.0f; // Convert to m/s

	// Convert pixel offsets to velocity commands
	float vel_y = -kp_azimuth * off_azimuth;   // Azimuth -> Y velocity
	float vel_x = -kp_elevation * off_elevation; // Elevation -> X velocity

	// Limit velocity
	float vel_magnitude = sqrtf(vel_x * vel_x + vel_y * vel_y);
	if (vel_magnitude > max_velocity) {
		vel_x = vel_x * max_velocity / vel_magnitude;
		vel_y = vel_y * max_velocity / vel_magnitude;
	}

	_current_velocity = matrix::Vector2f(vel_x, vel_y);

	// 对于速度控制，不需要设置位置目标
	_position_setpoint_valid = true;
	PX4_INFO("Calculated velocity: vx=%.3f, vy=%.3f",
		(double)_current_velocity(0), (double)_current_velocity(1));
}

void VisionGuidance::activate_guidance()
{
	PX4_INFO("Activating vision guidance");
	_guidance_active = true;
	_activation_time = hrt_absolute_time();
	set_guidance_state(GuidanceState::ACTIVE);
}

void VisionGuidance::deactivate_guidance()
{
	PX4_INFO("Deactivating vision guidance - target lost");
	_guidance_active = false;
	_current_velocity = matrix::Vector2f(0.0f, 0.0f);
	_position_setpoint_valid = false;
	set_guidance_state(GuidanceState::LOST_TARGET);
}

bool VisionGuidance::safety_checks_passed()
{
	if (!is_vehicle_ready()) {
		PX4_WARN("Safety check failed: Vehicle not ready");
		return false;
	}

	if (!is_position_valid()) {
		PX4_WARN("Safety check failed: Position not valid");
		return false;
	}

	// 移除等待时间检查，或者大幅减少最小等待时间
	PX4_INFO("All safety checks passed");
	return true;
}

bool VisionGuidance::is_vehicle_ready()
{
	actuator_armed_s armed;
	vehicle_control_mode_s control_mode;

	bool armed_ok = _actuator_armed_sub.copy(&armed) && armed.armed;
	bool control_ok = _vehicle_control_mode_sub.copy(&control_mode) &&
			control_mode.flag_control_position_enabled;

	// PX4_INFO("Vehicle ready check: armed=%s, position_control=%s",
	// 	armed_ok ? "YES" : "NO", control_ok ? "YES" : "NO");

	return armed_ok && control_ok;
}

bool VisionGuidance::is_position_valid()
{
	vehicle_local_position_s local_pos;
	bool pos_ok = _vehicle_local_position_sub.copy(&local_pos) && local_pos.xy_valid;

	PX4_INFO("Position valid: %s", pos_ok ? "YES" : "NO");
	return pos_ok;
}

void VisionGuidance::set_guidance_state(GuidanceState new_state)
{
	if (_current_state != new_state) {
		PX4_INFO("Guidance state changed: %d -> %d", (int)_current_state, (int)new_state);
		_current_state = new_state;
	}
}

void VisionGuidance::update_guidance_status()
{
	vision_guidance_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.guidance_state = (uint8_t)_current_state;
	status.target_locked = is_target_locked(_last_pod_data);
	status.guidance_active = _guidance_active;
	status.off_target_azimuth = _last_pod_data.off_target_azimuth;
	status.off_target_elevation = _last_pod_data.off_target_elevation;
	status.velocity_x = _current_velocity(0);
	status.velocity_y = _current_velocity(1);

	_vision_guidance_status_pub.publish(status);
}

void VisionGuidance::Run()
{
	// Open UART if not already open
	if (_uart_fd < 0) {
		if (open_uart() != 0) {
		return;
		}
	}

	// Read data from UART
	uint8_t buffer[64];
	int bytes_read = read_uart_data(buffer, sizeof(buffer));

	if (bytes_read == sizeof(VisionPodData)) {
		VisionPodData pod_data;
		memcpy(&pod_data, buffer, sizeof(VisionPodData));

		if (validate_frame(pod_data)) {
		process_pod_data(pod_data);
		} else {
		PX4_DEBUG("Invalid frame received");
		}
	} else if (bytes_read > 0) {
		PX4_DEBUG("Received %d bytes, expected %zu", bytes_read, sizeof(VisionPodData));
	}

	// Check for data timeout
	if (hrt_elapsed_time(&_last_valid_data) > DATA_TIMEOUT_US) {
		if (_guidance_active) {
		PX4_WARN("Vision data timeout - deactivating guidance");
		deactivate_guidance();
		}
		set_guidance_state(GuidanceState::WAITING);
	}

    // Publish velocity setpoint if guidance is active
	if (_guidance_active) {
	position_setpoint_triplet_s pos_sp_triplet{};
	pos_sp_triplet.timestamp = hrt_absolute_time();
	pos_sp_triplet.current.timestamp = hrt_absolute_time();
	pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_VELOCITY;
	pos_sp_triplet.current.valid = true;

	// 设置速度控制
	pos_sp_triplet.current.vx = _current_velocity(0);
	pos_sp_triplet.current.vy = _current_velocity(1);
	pos_sp_triplet.current.vz = 0.0f;  // 保持高度不变

	// 对于速度控制，位置信息设置为NaN
	pos_sp_triplet.current.lat = static_cast<double>(NAN);
	pos_sp_triplet.current.lon = static_cast<double>(NAN);
	pos_sp_triplet.current.alt = NAN;  // 高度设为NaN，让飞控保持当前高度
	pos_sp_triplet.current.yaw = NAN;

	pos_sp_triplet.current.cruising_speed = -1.0f; // use default
	pos_sp_triplet.current.cruising_throttle = -1.0f; // use default

	_pos_sp_triplet_pub.publish(pos_sp_triplet);
	} else if (_position_setpoint_valid) {
        // 当退出引导时，发布停止指令
        position_setpoint_triplet_s pos_sp_triplet{};
        pos_sp_triplet.timestamp = hrt_absolute_time();
        pos_sp_triplet.current.timestamp = hrt_absolute_time();
        pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
        pos_sp_triplet.current.valid = false; // 设置为无效，让飞行器保持当前位置

        _pos_sp_triplet_pub.publish(pos_sp_triplet);
        _position_setpoint_valid = false;
}
}

#include <cinttypes>
int VisionGuidance::print_status()
{
	PX4_INFO("Vision Guidance Status:");
	PX4_INFO("  State: %d", (int)_current_state);
	PX4_INFO("  Guidance Active: %s", _guidance_active ? "Yes" : "No");
	PX4_INFO("  UART Port: %s", _uart_port);
	PX4_INFO("  Last Data: %" PRIu64 " ms ago", hrt_elapsed_time(&_last_valid_data) / 1000);

	if (_guidance_active) {
		PX4_INFO("  Current Velocity: %.2f, %.2f m/s",
		(double)_current_velocity(0), (double)_current_velocity(1));
	}

	return 0;
}

int VisionGuidance::task_spawn(int argc, char *argv[])
{
	VisionGuidance *instance = new VisionGuidance();

	if (!instance) {
		PX4_ERR("alloc failed");
		return -1;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	instance->ScheduleOnInterval(40); // 25Hz - matches pod data rate

	return 0;
}

int VisionGuidance::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int VisionGuidance::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
	### Description
	Vision Guidance module for target tracking and terminal guidance.
	Listens to vision pod data over UART and controls UAV to track targets.

	### Implementation
	- Polls UART for vision pod data at 25Hz
	- Processes target offset information
	- Controls UAV position based on target tracking
	- Multiple safety checks and state management

	### Examples
	$ vision_guidance start
	)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("vision_guidance", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int vision_guidance_main(int argc, char *argv[])
{
	return VisionGuidance::main(argc, argv);
}
