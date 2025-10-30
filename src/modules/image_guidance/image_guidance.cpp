#include "image_guidance.h"
#include <px4_platform_common/log.h>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <mathlib/mathlib.h>           // 另一个可能的位置
#include <lib/geo/geo.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/tasks.h>


ImageGuidance::ImageGuidance() : Device("image_guidance") {
	PX4_INFO("Image guidance module started");
    // 串口路径在init()中通过IMAGE_GUIDANCE_UART_PATH处理
}

ImageGuidance::~ImageGuidance()
{
	PX4_INFO("Image guidance module stopped");
	if (_uart_fd >= 0) {
		close(_uart_fd);
	}

	perf_free(_loop_perf);
	perf_free(_serial_errors);
}

int ImageGuidance::init()
{
	PX4_INFO("Image guidance module initializing");
	int ret = open_serial_port();
	if (ret != PX4_OK) {
		PX4_ERR("Failed to initialize UART");
		perf_free(_loop_perf);
		perf_free(_serial_errors);
		return ret;
	}

    	// Initialize control setpoint structure
	memset(&_control_setpoint, 0, sizeof(_control_setpoint));
	_control_setpoint.timestamp = hrt_absolute_time();

	PX4_INFO("Image guidance module initialized");
	return PX4_OK;
}

int ImageGuidance::open_serial_port()
{
	PX4_INFO("IMAGE_GUIDANCE_UART_PATH: %s", IMAGE_GUIDANCE_UART_PATH);
	PX4_INFO("this is open_serial_port");
	_uart_fd = open(IMAGE_GUIDANCE_UART_PATH, O_RDWR | O_NOCTTY);
	PX4_INFO("_uart_fd: %d", _uart_fd);
	if (_uart_fd < 0) {
        PX4_ERR("Failed to open UART: %s (errno: %d)", IMAGE_GUIDANCE_UART_PATH, errno);
        return -errno;
	}

	// 新增：检查文件描述符是否有效
	if (fcntl(_uart_fd, F_GETFL) < 0) {
		PX4_ERR("UART file descriptor is invalid (errno: %d)", errno);
		close(_uart_fd);
		return -errno;
	}

	struct termios uart_config;
	memset(&uart_config, 0, sizeof(uart_config));

	// Configure 8N1
	uart_config.c_cflag = CS8 | CLOCAL | CREAD;  // 8位数据位，忽略调制解调器控制，启用接收
	uart_config.c_iflag = IGNPAR;                // 忽略奇偶校验错误
	uart_config.c_oflag = 0;                     // 无输出处理
	uart_config.c_lflag = 0;                     // 无本地模式处理
	uart_config.c_cc[VTIME] = 0;                 // 无超时
	uart_config.c_cc[VMIN] = 1;                  // 阻塞读取，直到至少1字节数据

    	// Set baud rate
	if (cfsetispeed(&uart_config, BAUDRATE) < 0 || cfsetospeed(&uart_config, BAUDRATE) < 0) {
		PX4_ERR("Failed to set baud rate: %d (errno: %d)", BAUDRATE, errno);
		close(_uart_fd);
		return -errno;
	}

    	// Apply configuration
	if (tcsetattr(_uart_fd, TCSANOW, &uart_config) < 0) {
		PX4_ERR("Failed to apply UART configuration (errno: %d)", errno);
		close(_uart_fd);
		return -errno;
	}

	// 新增：刷新串口输入输出缓冲区
	tcflush(_uart_fd, TCIOFLUSH);

	// 新增：验证串口配置是否生效
	struct termios new_config;
	memset(&new_config, 0, sizeof(new_config));
	if (tcgetattr(_uart_fd, &new_config) < 0) {
		PX4_ERR("Failed to verify UART configuration (errno: %d)", errno);
		close(_uart_fd);
		return -errno;
	}

	// 删除以下配置比较代码块
	if (new_config.c_cflag != uart_config.c_cflag) {
		PX4_ERR("UART configuration mismatch");
		close(_uart_fd);
		return -EINVAL;
	}


	PX4_INFO("_uart_fd: %d", _uart_fd);

	// 打印波特率宏定义值（用于调试）
	PX4_INFO("BAUDRATE宏值: 0x%x (%d)", BAUDRATE, BAUDRATE);
	// 打印控制标志位（十六进制便于调试）
	PX4_INFO("uart_config.c_cflag: 0x%x (%d)", uart_config.c_cflag, uart_config.c_cflag);
	PX4_INFO("new_config.c_cflag: 0x%x (%d)", new_config.c_cflag, new_config.c_cflag);
	return PX4_OK;
}

void ImageGuidance::run()
{
	PX4_INFO("Image guidance module running");
	perf_begin(_loop_perf);

	// 修复：使用ModuleBase提供的退出机制


	if (_uart_fd < 0) {

		PX4_ERR("UART not initialized, exiting module");
		request_stop(); // 标准退出标志
		perf_end(_loop_perf);
		return;
	}
	PX4_INFO("_uart_fd: %d", _uart_fd);

	PX4_INFO("this is run");
	int ret = read_serial_data();

	// 检查返回值，如果出错则退出
	if (ret != PX4_OK) {
		request_stop();
		perf_end(_loop_perf);
		return;
	}

	// Only calculate control if tracking is active
	if (_tracking_active) {
		calculate_control();
		publish_control_setpoint();
	}

	perf_end(_loop_perf);
}

int ImageGuidance::read_serial_data()
{
	PX4_INFO("this is read_serial_data");
	static int call_counter = 0; // 初始化静态计数器
	call_counter++; // 每次调用递增计数器
	PX4_INFO("read_serial_data,this is call_counter: %d", call_counter);
	// 新增：检查文件描述符有效性
	if (_uart_fd < 0) {
		return -1;
	}

	uint8_t buf[128];
	ssize_t n = read(_uart_fd, buf, sizeof(buf));
	if (n < 0) {
		PX4_INFO("read_serial_data,this is nnnnn");
		PX4_ERR("UART read failed (errno: %d - %s)", errno, strerror(errno));
		return -errno;
	}
	int saved_errno = errno; // 立即保存错误码
	PX4_INFO("read_serial_data");
	PX4_INFO("n: %zd", n);


	if (n > 0) {
		// 打印十六进制数据（前16字节）
		char hex_buf[3*16 + 1] = {0};
	for (int i=0; i<math::min(n, static_cast<ssize_t>(16)); i++) {
		snprintf(hex_buf + 3*i, 4, "%02X ", buf[i]);
	}
		PX4_INFO("buf: %s", hex_buf);
	} else if (n == 0) {
		PX4_INFO("buf: (empty)");
	} else {
		PX4_INFO("read error: %d (%s)", saved_errno, strerror(saved_errno));
	}

	PX4_DEBUG("UART received %zd bytes", n);

	if (n < 0) {
		if (saved_errno == EINVAL) {
			PX4_ERR("UART read error: Invalid argument (EINVAL). Check UART configuration.");
			request_stop();
			return -saved_errno;
		}
		#if EAGAIN == EWOULDBLOCK
		if (saved_errno != EAGAIN) {
		#else
		if (saved_errno != EAGAIN && saved_errno != EWOULDBLOCK) {
		#endif
		perf_count(_serial_errors);
		PX4_ERR("UART read error: %d", saved_errno);
		}
		return -saved_errno;
	}

	for (ssize_t i = 0; i < n; i++) {
		if (!parse_frame(buf[i])) {
		perf_count(_serial_errors);
		}
	}

	return PX4_OK;
}

bool ImageGuidance::parse_frame(uint8_t data)
{
	// Simple state machine for frame parsing
	static enum {
		STATE_WAIT_HEADER1,
		STATE_WAIT_HEADER2,
		STATE_READ_DATA,
		STATE_WAIT_TAIL
	} state = STATE_WAIT_HEADER1;

	switch (state) {
		case STATE_WAIT_HEADER1:
		if (data == SYNC_HEADER1) {
			_rx_buf[0] = data;
			_rx_buf_idx = 1;
			state = STATE_WAIT_HEADER2;
		}
		break;

		case STATE_WAIT_HEADER2:
		if (data == SYNC_HEADER2) {
			_rx_buf[1] = data;
			_rx_buf_idx = 2;
			state = STATE_READ_DATA;
		} else {
			state = STATE_WAIT_HEADER1; // Reset
		}
		break;

		case STATE_READ_DATA:
		_rx_buf[_rx_buf_idx++] = data;

		// Check if we've read enough data
		if (_rx_buf_idx >= MAX_FRAME_LENGTH - 1) {
			state = STATE_WAIT_TAIL;
		}
		break;

		case STATE_WAIT_TAIL:
		if (data == FRAME_TAIL) {
			_rx_buf[_rx_buf_idx] = data;
			state = STATE_WAIT_HEADER1;

			// Copy to feedback frame structure
			memcpy(&_feedback_frame, _rx_buf, sizeof(PodFeedbackFrame));

			// Verify checksum
			uint8_t checksum = 0;
			for (int i = 2; i < 62; i++) { // Bytes 3-62 (0-based index 2-61)
			checksum ^= _rx_buf[i];
			}

			if (checksum == _feedback_frame.checksum) {
			// Update tracking status based on pod status
			_tracking_active = (_feedback_frame.pod_status1 & 0x0600) == 0x0200;
			_azimuth_offset = _feedback_frame.offset_azimuth;
			_pitch_offset = _feedback_frame.offset_pitch;

			// 新增：打印原始数据
			PX4_INFO("Pod Data: status=0x%04X, azimuth=%d, pitch=%d, checksum=0x%02X",
			_feedback_frame.pod_status1, _azimuth_offset, _pitch_offset, checksum);

			return true;
			}

		} else {
			state = STATE_WAIT_HEADER1; // Invalid tail, reset
		}
		break;

		default:
		state = STATE_WAIT_HEADER1;
		break;
	}

	return false;
}

void ImageGuidance::calculate_control()
{
	// Simple proportional control for demonstration
	const float Kp_azimuth = 0.001f; // Adjust based on system response
	const float Kp_pitch = 0.001f;

	// Convert pixel offsets to control inputs (normalized to [-1, 1])
	_control_setpoint.roll = math::constrain(_azimuth_offset * Kp_azimuth, -1.0f, 1.0f);
	_control_setpoint.pitch = math::constrain(_pitch_offset * Kp_pitch, -1.0f, 1.0f);

	// 打印控制量计算结果（修复类型转换错误）
	PX4_INFO("Control Calculation: azim_offset=%d(px) -> roll=%.3f, pitch_offset=%d(px) -> pitch=%.3f",
			_azimuth_offset, (double)_control_setpoint.roll,
			_pitch_offset, (double)_control_setpoint.pitch);

	// Keep altitude and yaw unchanged
	_control_setpoint.throttle = NAN;
	_control_setpoint.yaw= NAN;

	_control_setpoint.timestamp = hrt_absolute_time();
}

void ImageGuidance::publish_control_setpoint()
{
	_control_setpoint.timestamp = hrt_absolute_time();

	// 新增：打印发布的控制量
	PX4_INFO("Publishing Control: roll=%.3f, pitch=%.3f",
		(double)_control_setpoint.roll, (double)_control_setpoint.pitch);

	_manual_control_pub.publish(_control_setpoint);
}

int ImageGuidance::print_status()
{
	PX4_INFO("Image Guidance Module Status:");
	PX4_INFO("UART FD: %d", _uart_fd);
	PX4_INFO("Tracking Active: %s", _tracking_active ? "Yes" : "No");
	PX4_INFO("Azimuth Offset: %d, Pitch Offset: %d", _azimuth_offset, _pitch_offset);
	PX4_INFO("B115200 value: %d, B230400 value: %d", B115200, B230400);
	perf_print_counter(_loop_perf);
	perf_print_counter(_serial_errors);
	return 0;
}

bool ImageGuidance::should_exit() const {
	return ModuleBase<ImageGuidance>::should_exit();
}


// Module initialization function
extern "C" __EXPORT int image_guidance_main(int argc, char *argv[])
{
	PX4_INFO("Starting Image Guidance Module");

	ImageGuidance guidance;

	if (guidance.init() != PX4_OK) {
		PX4_ERR("Image guidance initialization failed");
		return 1;
	}

    // Run module loop at 50Hz
//     while (!guidance.is_running()) {  // 替换为 guidance.should_exit()
//         guidance.run();
//         px4_usleep(20000); // 50Hz
//     }
	while (!guidance.should_exit()) {
		guidance.run();
		// 新增：检查退出请求
		if (guidance.should_exit()) {
			break;
		}
		px4_usleep(200000); // 50Hz
	}

	PX4_INFO("Exiting Image Guidance Module");
	return 0;
}
