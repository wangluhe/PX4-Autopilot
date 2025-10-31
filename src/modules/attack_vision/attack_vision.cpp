#include "attack_vision.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

AttackVision::AttackVision()
	: ModuleParams(nullptr)
	, WorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

AttackVision::~AttackVision()
{
	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}
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

int AttackVision::print_status()
{
	PX4_INFO("attack_vision running, fd=%d, lock=%d, pix=(%d,%d)", _fd, (int)_lock_active, (int)_pix_offset_x, (int)_pix_offset_y);
	return 0;
}

bool AttackVision::configure_uart(int baudrate)
{
	if (_fd < 0) { return false; }

	termios t{};
	if (tcgetattr(_fd, &t) != 0) {
		PX4_ERR("tcgetattr failed");
		return false;
	}
	cfmakeraw(&t);
	t.c_cflag |= CLOCAL | CREAD;
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
	const char *dev = "/dev/ttyS2"; // TELEM2 default
	_fd = ::open(dev, O_RDWR | O_NOCTTY);
	if (_fd < 0) {
		PX4_ERR("open %s failed", dev);
		return false;
	}
	return configure_uart(_param_av_baud.get());
}

bool AttackVision::validate_frame(const uint8_t *frame)
{
	if (!frame) return false;
	if (frame[0] != FRAME_HEAD_0 || frame[1] != FRAME_HEAD_1) return false;
	if (frame[63] != FRAME_TAIL) return false;
	uint8_t xorv = 0;
	for (int i = 2; i <= 61; i++) { xorv ^= frame[i]; }
	return xorv == frame[62];
}

bool AttackVision::try_read_frame()
{
	if (_fd < 0) return false;
	uint8_t byte;
	ssize_t n = ::read(_fd, &byte, 1);
	if (n != 1) return false;

	if (_buf_len == 0) {
		if (byte != FRAME_HEAD_0) return false;
	}
	_buf[_buf_len++] = byte;
	if (_buf_len == 1) return false;
	if (_buf_len == 2 && _buf[1] != FRAME_HEAD_1) { _buf_len = 0; return false; }
	if (_buf_len < FRAME_LEN) return false;

	bool ok = validate_frame(_buf);
	if (!ok) {
		// shift one byte and continue searching
		memmove(_buf, _buf + 1, FRAME_LEN - 1);
		_buf_len = FRAME_LEN - 1;
		return false;
	}

	_last_frame_time_us = hrt_absolute_time();

	// decode fields
	uint16_t status_5_6 = (uint16_t)_buf[4] | ((uint16_t)_buf[5] << 8);
	uint8_t servo_state = _buf[8];
	uint16_t lock_bits = (status_5_6 >> 9) & 0x3; // Bit9~Bit10
	bool locking = (lock_bits == 0x1) || (lock_bits == 0x2);
	bool exit_lock = (lock_bits == 0x3);
	_lock_active = locking && (servo_state == 0x07);
	if (exit_lock) { _lock_active = false; }

	_pix_offset_x = (int16_t)((uint16_t)_buf[58] | ((uint16_t)_buf[59] << 8));
	_pix_offset_y = (int16_t)((uint16_t)_buf[60] | ((uint16_t)_buf[61] << 8));

	_buf_len = 0; // reset for next frame
	return true;
}

bool AttackVision::switch_to_offboard()
{
	vehicle_status_s vs{};
	if (_vehicle_status_sub.copy(&vs)) {
		if (vs.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD && vs.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
			return true;
		}
	}

	vehicle_command_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.param1 = 1; // custom
	cmd.param2 = 6; // main mode offboard
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	cmd.target_system = 1;
	cmd.target_component = 1;
	_vehicle_cmd_pub.publish(cmd);
	return false;
}

void AttackVision::switch_to_hold()
{
	vehicle_command_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.param1 = 1; // custom
	cmd.param2 = 4; // main mode auto
	cmd.param3 = 3; // submode loiter
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	cmd.target_system = 1;
	cmd.target_component = 1;
	_vehicle_cmd_pub.publish(cmd);
}

void AttackVision::publish_offboard_velocity(float vx, float vy, float vz, float yaw_rate)
{
	offboard_control_mode_s ocm{};
	ocm.timestamp = hrt_absolute_time();
	ocm.position = false;
	ocm.velocity = true;
	ocm.acceleration = false;
	ocm.attitude = false;
	ocm.body_rate = false;
	ocm.actuator = false;
	_offboard_ctrl_pub.publish(ocm);

	trajectory_setpoint_s sp{};
	sp.timestamp = ocm.timestamp;
	sp.vx = vx;
	sp.vy = vy;
	sp.vz = vz;
	sp.yawspeed = yaw_rate;
	_traj_sp_pub.publish(sp);
}

void AttackVision::handle_guidance()
{
	// compute velocity commands from pixel offsets
	const float kp = _param_av_kp.get();
	const float dead = _param_av_dead.get();
	const float maxv = _param_av_max_v.get();

	float ex = (float)_pix_offset_x;
	float ey = (float)_pix_offset_y;
	if (PX4_ISFINITE(ex) && PX4_ISFINITE(ey)) {
		if (fabsf(ex) < dead) ex = 0.f;
		if (fabsf(ey) < dead) ey = 0.f;
		float vx = -px4::constrain_float(kp * ex, -maxv, maxv);
		float vy = -px4::constrain_float(kp * ey, -maxv, maxv);
		publish_offboard_velocity(vx, vy, 0.f, 0.f);
	}
}

void AttackVision::Run()
{
	if (_param_av_en.get() <= 0) {
		PX4_WARN("AV_EN disabled");
		exit_and_cleanup();
		return;
	}
	if (!open_uart()) {
		PX4_ERR("UART open failed");
		exit_and_cleanup();
		return;
	}

	px4_pollfd_struct_t p{};
	p.fd = _fd;
	p.events = POLLIN;

	const uint64_t frame_timeout_us = 200000; // 200ms
	while (!should_exit()) {
		int ret = px4_poll(&p, 1, 20);
		if (ret > 0 && (p.revents & POLLIN)) {
			while (try_read_frame()) {
				// process immediately
			}
		}

		bool frame_valid_recent = (hrt_absolute_time() - _last_frame_time_us) < frame_timeout_us;
		if (_lock_active && frame_valid_recent) {
			if (switch_to_offboard()) {
				handle_guidance();
			}
		} else {
			// hover/hold
			switch_to_hold();
		}
	}

	exit_and_cleanup();
}

extern "C" __EXPORT int attack_vision_main(int argc, char *argv[])
{
	return AttackVision::main(argc, argv);
}


