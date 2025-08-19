#include "image_guidance.h"
#include <px4_platform_common/log.h>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

ImageGuidance::ImageGuidance() : Device(nullptr, DEVICE_PATH) {}

ImageGuidance::~ImageGuidance()
{
    if (_uart_fd >= 0) {
        close(_uart_fd);
    }

    perf_free(_loop_perf);
    perf_free(_serial_errors);
}

int ImageGuidance::init()
{
    int ret = open_serial_port();
    if (ret != PX4_OK) {
        PX4_ERR("Failed to initialize UART");
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
    _uart_fd = open(IMAGE_GUIDANCE_UART_PATH, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_uart_fd < 0) {
        PX4_ERR("Failed to open UART: %s", IMAGE_GUIDANCE_UART_PATH);
        return -errno;
    }

    struct termios uart_config;
    memset(&uart_config, 0, sizeof(uart_config));

    // Configure 8N1
    uart_config.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    uart_config.c_iflag = IGNPAR;
    uart_config.c_oflag = 0;
    uart_config.c_lflag = 0;
    uart_config.c_cc[VTIME] = 0;
    uart_config.c_cc[VMIN] = 1;

    // Set baud rate
    if (cfsetispeed(&uart_config, BAUDRATE) < 0 || cfsetospeed(&uart_config, BAUDRATE) < 0) {
        PX4_ERR("Failed to set baud rate");
        close(_uart_fd);
        return -errno;
    }

    // Apply configuration
    if (tcsetattr(_uart_fd, TCSANOW, &uart_config) < 0) {
        PX4_ERR("Failed to apply UART configuration");
        close(_uart_fd);
        return -errno;
    }

    return PX4_OK;
}

void ImageGuidance::run()
{
    perf_begin(_loop_perf);

    read_serial_data();

    // Only calculate control if tracking is active
    if (_tracking_active) {
        calculate_control();
        publish_control_setpoint();
    }

    perf_end(_loop_perf);
}

int ImageGuidance::read_serial_data()
{
    uint8_t buf[128];
    ssize_t n = read(_uart_fd, buf, sizeof(buf));

    if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            perf_count(_serial_errors);
            PX4_ERR("UART read error: %d", errno);
        }
        return -errno;
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
                    _tracking_active = (_feedback_frame.pod_status1 & 0x0600) == 0x0200; // Bit9-10: 01=锁定中
                    _azimuth_offset = _feedback_frame.offset_azimuth;
                    _pitch_offset = _feedback_frame.offset_pitch;
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
    _control_setpoint.x = constrain_float(_azimuth_offset * Kp_azimuth, -1.0f, 1.0f);
    _control_setpoint.y = constrain_float(_pitch_offset * Kp_pitch, -1.0f, 1.0f);

    // Keep altitude and yaw unchanged
    _control_setpoint.z = NAN;
    _control_setpoint.r = NAN;

    _control_setpoint.timestamp = hrt_absolute_time();
}

void ImageGuidance::publish_control_setpoint()
{
    _control_setpoint.timestamp = hrt_absolute_time();
    _manual_control_pub.publish(_control_setpoint);
}

void ImageGuidance::print_status()
{
    PX4_INFO("Image Guidance Module Status:");
    PX4_INFO("UART FD: %d", _uart_fd);
    PX4_INFO("Tracking Active: %s", _tracking_active ? "Yes" : "No");
    PX4_INFO("Azimuth Offset: %d, Pitch Offset: %d", _azimuth_offset, _pitch_offset);
    perf_print_counter(_loop_perf);
    perf_print_counter(_serial_errors);
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
    while (!px4_should_exit()) {
        guidance.run();
        px4_usleep(20000); // 50Hz
    }

    PX4_INFO("Exiting Image Guidance Module");
    return 0;
}
