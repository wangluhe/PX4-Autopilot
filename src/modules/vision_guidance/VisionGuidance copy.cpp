#include "VisionGuidance.hpp"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#include <fcntl.h>
#include <unistd.h>
#include <poll.h>

// 在构造函数中初始化参数
VisionGuidance::VisionGuidance() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	// 从参数中获取UART端口配置
	switch (_param_uart_port.get()) {
		case 0: _uart_port = "/dev/ttyS0"; break;
		case 1: _uart_port = "/dev/ttyS1"; break;
		case 2: _uart_port = "/dev/ttyS2"; break;
		case 3: _uart_port = "/dev/ttyS3"; break;
		case 4: _uart_port = "/dev/ttyS4"; break;
		case 5: _uart_port = "/dev/ttyS5"; break;
		case 6: _uart_port = "/dev/ttyS6"; break;
		default: _uart_port = "/dev/ttyS2"; break;
	}

	_baudrate = _param_baudrate.get();

	// Initialize parameter update
	updateParams();
}

VisionGuidance::~VisionGuidance()
{
	close_uart();
}

int VisionGuidance::open_uart()
{
	_uart_fd = ::open(_uart_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_uart_fd < 0) {
		PX4_ERR("failed to open UART port %s", _uart_port);
		return -1;
	}

	return setup_uart();
}

int VisionGuidance::setup_uart()
{
	struct termios uart_config;
	int termios_state;

	// Back up original configuration
	if ((termios_state = tcgetattr(_uart_fd, &uart_config)) < 0) {
		PX4_ERR("tcgetattr failed for %s: %d", _uart_port, termios_state);
		::close(_uart_fd);
		_uart_fd = -1;
		return -1;
}

    // Clear current configuration
	tcflush(_uart_fd, TCIOFLUSH);

    // Set baudrate
	speed_t speed;
	switch (_baudrate) {
		case 9600:   speed = B9600; break;
		case 19200:  speed = B19200; break;
		case 38400:  speed = B38400; break;
		case 57600:  speed = B57600; break;
		case 115200: speed = B115200; break;
		case 230400: speed = B230400; break;
		case 460800: speed = B460800; break;
		case 921600: speed = B921600; break;
		default:     speed = B115200; break;
	}

	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		PX4_ERR("failed to set baudrate for %s", _uart_port);
		::close(_uart_fd);
		_uart_fd = -1;
		return -1;
	}

	// Configure UART
	uart_config.c_cflag |= (CLOCAL | CREAD);    // Ignore modem controls
	uart_config.c_cflag &= ~CSIZE;
	uart_config.c_cflag |= CS8;                 // 8-bit characters
	uart_config.c_cflag &= ~PARENB;             // No parity
	uart_config.c_cflag &= ~CSTOPB;             // 1 stop bit
	uart_config.c_cflag &= ~CRTSCTS;            // No hardware flow control

    // Input modes
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);

	// Output modes
	uart_config.c_oflag &= ~OPOST;

	// Local modes
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

	// Special characters
	uart_config.c_cc[VMIN] = 0;
	uart_config.c_cc[VTIME] = 10; // 1 second timeout

	if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("tcsetattr failed for %s: %d", _uart_port, termios_state);
		::close(_uart_fd);
		_uart_fd = -1;
		return -1;
	}

	tcflush(_uart_fd, TCIOFLUSH);
	PX4_INFO("UART %s configured with baudrate %d", _uart_port, _baudrate);
	return 0;
}

void VisionGuidance::close_uart()
{
	if (_uart_fd >= 0) {
		::close(_uart_fd);
		_uart_fd = -1;
	}
}

int VisionGuidance::read_uart_data(uint8_t *buffer, size_t length)
{
	if (_uart_fd < 0) {
		return -1;
	}

	struct pollfd fds[1];
	fds[0].fd = _uart_fd;
	fds[0].events = POLLIN;

	int ret = poll(fds, 1, 10); // 10ms timeout

	if (ret > 0) {
		if (fds[0].revents & POLLIN) {
		return ::read(_uart_fd, buffer, length);
		}
	}

	return 0;
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
	return (lock_status == 0x01 || lock_status == 0x02); // 01:锁定中, 10:锁定预测
}

void VisionGuidance::process_pod_data(const VisionPodData &data)
{
	_last_valid_data = hrt_absolute_time();
	_last_pod_data = data;

	// Check if target is locked and guidance should be activated
	bool target_locked = is_target_locked(data);

	if (target_locked && !_guidance_active) {
		if (safety_checks_passed()) {
		activate_guidance();
		}
	} else if (!target_locked && _guidance_active) {
		deactivate_guidance();
	}

	// If guidance is active, calculate control commands
	if (_guidance_active) {
		calculate_guidance_commands(data.off_target_azimuth, data.off_target_elevation);
	}

	update_guidance_status();
}

// 在 calculate_guidance_commands 函数中更新参数使用：
void VisionGuidance::calculate_guidance_commands(int16_t off_azimuth, int16_t off_elevation)
{
	// Convert integer parameters to float (divide by scaling factor)
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
		PX4_WARN("Vehicle not ready for guidance");
		return false;
	}

	if (!is_position_valid()) {
		PX4_WARN("Position not valid for guidance");
		return false;
	}

	// Check if we've been in WAITING state for sufficient time
	if (_current_state == GuidanceState::WAITING &&
		hrt_elapsed_time(&_last_valid_data) < MIN_ACTIVATION_TIME_US) {
		return false;
	}

	return true;
}

bool VisionGuidance::is_vehicle_ready()
{
	actuator_armed_s armed;
	vehicle_control_mode_s control_mode;

	return (_actuator_armed_sub.copy(&armed) && armed.armed &&
		_vehicle_control_mode_sub.copy(&control_mode) && control_mode.flag_control_position_enabled);
}

bool VisionGuidance::is_position_valid()
{
	vehicle_local_position_s local_pos;
	return (_vehicle_local_position_sub.copy(&local_pos) && local_pos.xy_valid);
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
		pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_VELOCITY; // 使用速度控制
		pos_sp_triplet.current.valid = true;

		// 设置速度控制
		pos_sp_triplet.current.vx = _current_velocity(0);
		pos_sp_triplet.current.vy = _current_velocity(1);
		pos_sp_triplet.current.vz = 0.0f;

		// 对于速度控制，位置信息设置为NaN
		pos_sp_triplet.current.lat = static_cast<double>(NAN);
		pos_sp_triplet.current.lon = static_cast<double>(NAN);
		pos_sp_triplet.current.alt = NAN;
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
