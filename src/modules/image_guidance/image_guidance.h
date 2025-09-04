#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/module.h>
#include <drivers/device/device.h>
#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <termios.h>  // 确保此头文件已包含

#define IMAGE_GUIDANCE_UART_PATH "/dev/pts/1"  // 将物理串口改为虚拟串口
// #define IMAGE_GUIDANCE_UART_PATH "/dev/ttyS1"
#define BAUDRATE B115200  // 使用标准波特率常量
// #define BAUDRATE 4097  // B115200的标准数值定义
#define MAX_FRAME_LENGTH 64
#define SYNC_HEADER1 0xFC
#define SYNC_HEADER2 0x2C
#define FRAME_TAIL 0xF0

#pragma pack(push, 1)
struct PodFeedbackFrame {
    uint8_t header1;          // 帧头1: 0xFC
    uint8_t header2;          // 帧头2: 0x2C
    uint8_t payload_type;     // 载荷类型
    uint8_t self_test;        // 自检结果
    uint16_t pod_status1;     // 吊舱状态1
    uint16_t pod_status2;     // 吊舱状态2
    uint8_t servo_status;     // 伺服状态

    int16_t azimuth_angle;    // 方位角 (0.01°)
    int16_t pitch_angle;      // 俯仰角 (0.01°)
    int16_t roll_angle;       // 横滚角 (0.01°)
    uint8_t reserved1[3];     // 备用字段

    uint8_t target_info;      // 目标编号及类型
    uint8_t reserved2[2];     // 备用字段
    uint8_t tf_usage;         // TF卡使用百分比
    uint8_t tf_capacity;      // TF卡总容量
    uint16_t ir_focal_length; // 红外焦距
    uint16_t vis_focal_length; // 可见光焦距
    uint16_t target_ids;      // 目标识别编号

    float target_lon;         // 目标经度
    float target_lat;         // 目标纬度
    int16_t target_alt;       // 目标海拔
    int16_t soc_temp;         // SOC温度
    int16_t gyro_azimuth;     // 方位角速度
    int16_t gyro_pitch;       // 俯仰角速度
    int16_t gyro_roll;        // 横滚角速度
    uint8_t display_mode;     // 显示模式
    uint8_t reserved3[2];     // 备用字段
    int16_t track_point_x;    // 跟踪点X坐标
    int16_t track_point_y;    // 跟踪点Y坐标
    int16_t track_width;      // 跟踪框宽度
    int16_t track_height;     // 跟踪框高度
    int16_t offset_azimuth;   // 方位脱靶量 (像素)
    int16_t offset_pitch;     // 俯仰脱靶量 (像素)
    uint8_t checksum;         // 校验和
    uint8_t tail;             // 帧尾: 0xF0
};
#pragma pack(pop)
// 在类定义开始前添加前置声明
class ImageGuidanceTest;

class ImageGuidance : public ModuleBase<ImageGuidance>, public device::Device
{
	friend class ImageGuidanceTest;
	friend class ImageGuidanceTestable;

public:

	ImageGuidance();
	~ImageGuidance() override;
	int init() override;
	void run() override;

	int print_status() override;
	bool should_exit() const;


private:
	int open_serial_port();
	int read_serial_data();
	bool parse_frame(uint8_t data);
        void calculate_control();
        void publish_control_setpoint();

        int _uart_fd{-1};
        char _rx_buf[MAX_FRAME_LENGTH] = {0};
        int _rx_buf_idx{0};
        PodFeedbackFrame _feedback_frame{};
        bool _tracking_active{false};
        int16_t _azimuth_offset{0};
        int16_t _pitch_offset{0};

        perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, "image_guidance_loop")};
        perf_counter_t _serial_errors{perf_alloc(PC_COUNT, "image_guidance_serial_errors")};

        uORB::Publication<manual_control_setpoint_s> _manual_control_pub{ORB_ID(manual_control_setpoint)};
        manual_control_setpoint_s _control_setpoint{};
};
