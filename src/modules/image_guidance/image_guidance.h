#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/posix.h>
#include <drivers/device/device.h>
#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/manual_control_setpoint.h>

#define IMAGE_GUIDANCE_UART_PATH "/dev/ttyS2"
#define BAUDRATE 115200
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

class ImageGuidance : public device::Device
{
public:
/**
 * @brief 构造函数，初始化图像引导模块。
 * @details 用于创建图像引导模块的实例，通常用于无人机或其他自主系统的视觉导航。
 */
	ImageGuidance();
/**
 * @brief 析构函数，用于释放 ImageGuidance 类实例占用的资源。
 * @override 表示此函数重写了基类的析构函数。
 */
	~ImageGuidance() override;

/**
 * @brief 初始化函数，用于执行组件的初始化操作。
 * @return 返回初始化结果，0表示成功，非0表示失败。
 * @override 表示此函数重写了基类的初始化函数。
 * @details 初始化函数用于执行组件的初始化操作，包括打开串口、读取串口数据、解析帧数据、计算控制量等。
 */
    int init() override;
/**
 * @brief 执行主运行逻辑。
 * @details 此函数负责启动并运行程序的核心功能，通常用于主循环或任务调度。
 */
    void run();
/**
 * @brief 打印当前状态信息。
 * @details 该函数用于输出对象的当前状态信息，通常用于调试或日志记录。
 * @note 这是一个虚函数，子类可以重写以实现自定义的状态打印逻辑。
 */
    void print_status();

// init()
// 初始化设备，包括打开串口和设置性能计数器。
// run()
// 主循环函数，负责读取串口数据、解析数据帧、计算控制指令并发布。
// parse_frame(uint8_t data)
// 解析从串口接收的数据帧，填充到 PodFeedbackFrame 结构体中。
// calculate_control()
// 根据解析的数据计算控制指令（如脱靶量）。
// publish_control_setpoint()
// 发布控制指令到系统中（通过 uORB 通信机制）。

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
// _feedback_frame
// 存储解析后的数据帧。
// _tracking_active
// 标志位，表示是否正在跟踪目标。
// _azimuth_offset 和 _pitch_offset
// 存储方位和俯仰的脱靶量（像素单位）。
// _manual_control_pub
// 用于发布控制指令的 uORB 发布者。
// 使用 _uart_fd 存储串口文件描述符， _rx_buf 存储接收到的数据。
