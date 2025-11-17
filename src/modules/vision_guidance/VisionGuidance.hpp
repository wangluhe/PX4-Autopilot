#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vision_guidance_status.h>
#include <uORB/topics/vehicle_command.h>

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>

#include <termios.h>

class VisionGuidance : public ModuleBase<VisionGuidance>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	VisionGuidance();
	~VisionGuidance() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void Run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;


private:
	// UART communication
	int _uart_fd{-1};
	const char *_uart_port{"/dev/ttyS5"};
	int _baudrate{115200};

	// Data structures
	#pragma pack(push, 1)
	struct VisionPodData {
		uint8_t header1;        // 0xFC
		uint8_t header2;        // 0x2C
		uint8_t payload_type;   // 载荷类型
		uint8_t self_test;      // 自检结果
		uint16_t status1;       // 吊舱状态1
		uint16_t status2;       // 吊舱状态2
		uint8_t servo_status;   // 伺服状态
		int16_t azimuth;        // 方位角 ×100
		int16_t elevation;      // 俯仰角 ×100
		int16_t roll;           // 横滚角 ×100
		uint8_t reserved1[3];   // 备用
		uint8_t target_info;    // 目标编号及类型
		uint8_t reserved2[2];   // 备用
		uint8_t tf_usage;       // TF使用容量百分比
		uint8_t tf_total;       // TF卡总容量
		uint16_t ir_focus;      // 红外焦距 ×10
		uint16_t visible_focus; // 可见光焦距 ×10
		uint16_t target_id;     // 目标识别编号
		float target_lon;       // 目标经度
		float target_lat;       // 目标纬度
		int16_t target_alt;     // 目标海拔高度
		int16_t soc_temp;       // SOC处理器温度
		int16_t gyro_azimuth;   // 陀螺方位角速度 ×100
		int16_t gyro_elevation; // 陀螺俯仰角速度 ×100
		int16_t gyro_roll;      // 陀螺横滚角速度 ×100
		uint8_t display_mode;   // 当前显示图像反馈
		uint8_t reserved3[2];   // 备用
		int16_t track_point_x;  // 目标跟踪点宽方向坐标
		int16_t track_point_y;  // 目标跟踪点高方向坐标
		int16_t track_box_w;    // 目标跟踪框宽方向长度
		int16_t track_box_h;    // 目标跟踪框高方向长度
		int16_t off_target_azimuth;  // 目标脱靶量方位方向
		int16_t off_target_elevation;// 目标脱靶量俯仰方向
		uint8_t checksum;       // 异或校验
		uint8_t footer;         // 0xF0
	};
	#pragma pack(pop)

	// Module states
	enum class GuidanceState {
		WAITING = 0,
		ACTIVE,
		LOST_TARGET,
		STATE_ERROR
	};

	// UART management
	int open_uart();
	void close_uart();
	int setup_uart();
	int read_uart_data(uint8_t *buffer, size_t length);
	bool validate_frame(const VisionPodData &data);
	uint8_t calculate_checksum(const VisionPodData &data);

	// Data processing
	void process_pod_data(const VisionPodData &data);
	void update_guidance_status();
	void calculate_guidance_commands(int16_t off_azimuth, int16_t off_elevation);

	// State management
	void set_guidance_state(GuidanceState new_state);
	bool is_target_locked(const VisionPodData &data);
	void activate_guidance();
	void deactivate_guidance();
	void emergency_stop();

	// Safety checks
	bool safety_checks_passed();
	bool is_vehicle_ready();
	bool is_position_valid();
	int start();
	bool _is_running{false};  // 添加这一行

	// Publications
	uORB::Publication<vision_guidance_status_s> _vision_guidance_status_pub{ORB_ID(vision_guidance_status)};
	uORB::Publication<position_setpoint_triplet_s> _pos_sp_triplet_pub{ORB_ID(position_setpoint_triplet)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};

	// Subscriptions
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};

	// // Module parameters
	// DEFINE_PARAMETERS(
	// 	(ParamFloat<px4::params::VISION_GUIDANCE_KP_AZ>) _param_kp_azimuth,
	// 	(ParamFloat<px4::params::VISION_GUIDANCE_KP_EL>) _param_kp_elevation,
	// 	(ParamFloat<px4::params::VISION_GUIDANCE_MAX_VEL>) _param_max_velocity,
	// 	(ParamInt<px4::params::VISION_GUIDANCE_UART_PORT>) _param_uart_port,
	// 	(ParamInt<px4::params::VISION_GUIDANCE_BAUDRATE>) _param_baudrate
	// )
	// 在参数定义部分更新为：
	DEFINE_PARAMETERS(
	(ParamInt<px4::params::VISION_G_KP_AZ>) _param_kp_azimuth,
	(ParamInt<px4::params::VISION_G_KP_EL>) _param_kp_elevation,
	(ParamInt<px4::params::VISION_G_MAX_VEL>) _param_max_velocity,
	(ParamInt<px4::params::VISION_UART_PORT>) _param_uart_port,
	(ParamInt<px4::params::VISION_BAUDRATE>) _param_baudrate
	)


	// Module state
	GuidanceState _current_state{GuidanceState::WAITING};
	hrt_abstime _last_valid_data{0};
	hrt_abstime _activation_time{0};
	VisionPodData _last_pod_data{};
	bool _guidance_active{false};

	// Control variables
	matrix::Vector2f _current_velocity{0.0f, 0.0f};
	matrix::Vector2f _target_position{0.0f, 0.0f};
	bool _position_setpoint_valid{false};

	static constexpr uint64_t DATA_TIMEOUT_US = 200000; // 200ms timeout
	static constexpr uint64_t MIN_ACTIVATION_TIME_US = 1000000; // 1 second minimum activation time
};
