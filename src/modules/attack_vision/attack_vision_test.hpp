/**
 * @file attack_vision.hpp
 * @brief 图像末制导模块头文件
 *
 * 该模块实现基于吊舱图像反馈的末制导功能：
 * - 通过串口接收吊舱64字节反馈帧
 * - 解析锁定状态和目标脱靶量（像素偏差）
 * - 当锁定有效时，自动切换到Offboard模式并发布速度控制指令
 * - 失锁或超时时，自动切换回悬停模式
 *
 * 硬件连接：吊舱串口连接飞控TELEM2口（/dev/ttyS2）
 */

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <matrix/matrix/math.hpp>
#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>  // 新增：订阅无人机姿态
#include <termios.h>
#include <poll.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/attack_vision_status.h>
#include <uORB/topics/external_mission_active.h>

/**
 * @class AttackVision
 * @brief 图像末制导模块主类
 *
 * 继承自ModuleBase、ModuleParams和WorkItem，实现PX4模块标准接口
 */
class AttackVision : public ModuleBase<AttackVision>, public ModuleParams, public px4::WorkItem
{
public:
	AttackVision();
	~AttackVision() override;

	// ModuleBase API
	static int task_spawn(int argc, char *argv[]);
	static AttackVision *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	static int soft_stop_command();
	static int resume_command();
	int print_status() override;
	void Run() override;

private:
	#ifdef __PX4_POSIX
		bool _virtual_gimbal_initialized{false};
		// 新增：固定的吊舱NED目标姿态（用于仿真）
		float _fixed_gimbal_roll_ned{0.0f};     // NED坐标系横滚角（弧度）
		float _fixed_gimbal_pitch_ned{0.0f};    // NED坐标系俯仰角（弧度）
		float _fixed_gimbal_yaw_ned{0.0f};      // NED坐标系偏航角（弧度）
	#endif


	// ========== 新增：RC遥控器相关 ==========
	enum class RCMode {
		MODE_OFFBOARD = 0,  // 末制导档位
		MODE_POSITION = 1,  // 定点档位
		MODE_ALTITUDE = 2,  // 定高档位
		MODE_UNKNOWN = 3    // 未知档位
	};
	uORB::Subscription _rc_input_sub{ORB_ID(input_rc)};  // 订阅RC输入话题
	RCMode _current_rc_mode{RCMode::MODE_UNKNOWN};       // 当前RC模式档位
	int _rc_mode_channel{6};                             // 模式选择通道（默认CH6）
	static constexpr int RC_PWM_MIN = 1000;              // RC通道最小PWM
	static constexpr int RC_PWM_MAX = 2000;              // RC通道最大PWM
	// 档位PWM阈值（可通过参数配置，这里先硬编码，后续可加参数）
	static constexpr int RC_THRESHOLD_OFFBOARD_LOW = 1000;
	static constexpr int RC_THRESHOLD_OFFBOARD_HIGH = 1300;
	static constexpr int RC_THRESHOLD_ALTITUDE_LOW = 1300;
	static constexpr int RC_THRESHOLD_ALTITUDE_HIGH = 1700;
	static constexpr int RC_THRESHOLD_POSITION_LOW = 1700;
	static constexpr int RC_THRESHOLD_POSITION_HIGH = 2000;
	// 新增：解析RC通道获取当前模式档位
	RCMode parse_rc_mode();
	// 新增：根据RC模式切换飞控模式
	void switch_to_rc_mode(RCMode target_mode);


	// ========== 串口通信相关 ==========
	int _fd{-1};  ///< 串口文件描述符
	bool configure_uart(int baudrate);  ///< 配置串口参数（波特率等）
	bool open_uart();  ///< 打开串口设备（默认/dev/ttyS2，TELEM2）

	// ========== 帧解析相关 ==========
	static constexpr int FRAME_LEN{64};  ///< 吊舱反馈帧固定长度：64字节
	static constexpr uint8_t FRAME_HEAD_0{0xFC};  ///< 帧头字节0
	static constexpr uint8_t FRAME_HEAD_1{0x2C};  ///< 帧头字节1
	static constexpr uint8_t FRAME_TAIL{0xF0};  ///< 帧尾字节
	uint8_t _buf[FRAME_LEN]{};  ///< 接收缓冲区
	int _buf_len{0};  ///< 当前缓冲区数据长度
	uint64_t _last_frame_time_us{0};  ///< 最后一次有效帧的时间戳（用于超时检测）
	bool try_read_frame();  ///< 尝试从串口读取一帧数据
	bool validate_frame(const uint8_t *frame);  ///< 校验帧格式（帧头、帧尾、异或校验）

	// ========== 解析出的关键数据 ==========
	bool _lock_active{false};  ///< 锁定状态：true表示正在锁定目标
	int16_t _pix_offset_x{0};  ///< 目标脱靶量-方位方向（像素，字节59-60，INT16）
	int16_t _pix_offset_y{0};  ///< 目标脱靶量-俯仰方向（像素，字节61-62，INT16）

	int16_t roll_deg_100{0}; // 横滚角（100倍度）
	int16_t pitch_deg_100{0}; // 俯仰角（100倍度）
	int16_t yaw_deg_100{0}; // 方位角（100倍度）

	float _gimbal_roll{0.0f};   // 吊舱横滚角（弧度）
	float _gimbal_pitch{0.0f};  // 吊舱俯仰角（弧度）
	float _gimbal_yaw{0.0f};    // 吊舱方位角（弧度）

	// ========== 状态机相关 ==========
	enum class ModuleState {
		HOLD,                   ///< 悬停模式
		SWITCHING_TO_OFFBOARD,  ///< 正在切换到Offboard模式
		OFFBOARD                ///< Offboard模式激活
	};
	ModuleState _module_state{ModuleState::HOLD};  ///< 当前模块状态

	// ========== 控制逻辑相关 ==========
	struct VehicleGuidanceState {
		matrix::Quatf q_veh_ned{};
		float veh_roll{0.0f};
		float veh_pitch{0.0f};
		float veh_yaw{0.0f};
		bool valid{false};
	};

	struct GimbalNedPose {
		matrix::Quatf q_gimbal_ned{};
		float gimbal_roll_ned{0.0f};
		float gimbal_pitch_ned{0.0f};
		float gimbal_yaw_ned{0.0f};
		bool valid{false};
	};

	struct GuidanceCommand {
		float target_roll{0.0f};
		float target_pitch{0.0f};
		float target_yaw{0.0f};
		float vx_ned{0.0f};
		float vy_ned{0.0f};
		float vz_ned{0.0f};
		float target_yaw_rate{0.0f};
		bool valid{false};
	};

	float _forward_velocity{1.0f};  ///< 恒定前向速度
	hrt_abstime _switch_start_time{0};  ///< 模式切换开始时间戳
	hrt_abstime _last_cmd_publish_time{0};  ///< 上次命令发布的时间戳（用于频率限制）
	static constexpr uint64_t MIN_CMD_INTERVAL_US = 500000;  ///< 命令发布最小间隔（500ms）
	void handle_guidance();  ///< 处理制导逻辑：根据像素偏差计算速度指令
	bool check_guidance_ready();
	bool read_vehicle_guidance_state(VehicleGuidanceState &state);
	bool get_gimbal_ned_pose(const matrix::Quatf &q_veh_ned, GimbalNedPose &pose);
	bool build_guidance_command(const VehicleGuidanceState &veh, const GimbalNedPose &gimbal, GuidanceCommand &cmd);
	void publish_guidance_command(const GuidanceCommand &cmd);
	void publish_offboard_velocity(float vx, float vy, float vz, float yaw_rate);  ///< 发布Offboard速度设定值
	void publish_position_offboard_heartbeat();  ///< 仅发布位置型 Offboard 心跳，让上位机/MAVROS位置设定点接管
	bool switch_to_offboard();  ///< 切换到Offboard模式（如果尚未切换）
	void switch_to_hold();  ///< 切换到悬停模式（Loiter）
	void safe_stop_guidance();  ///< stop/退出前释放Offboard控制并进入安全状态
	void reset_guidance_state();  ///< 清除末制导内部状态，避免重启继承旧数据
	void parse_frame_data();
	void close_uart();
	void print_drone_status();
	void publish_attitude_velocity_control(float target_roll, float target_pitch,
						float target_yaw, float vx_ned,
						float vy_ned, float vz_ned,
						float target_yaw_rate);
	bool switch_to_offboard_sim();  ///< 仿真模式下切换到Offboard模式（直接执行，跳过RC检查）

	// ========== uORB话题订阅和发布 ==========
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};  ///< 订阅载具状态
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};  // 订阅无人机当前姿态
	uORB::Publication<offboard_control_mode_s> _offboard_ctrl_pub{ORB_ID(offboard_control_mode)};  ///< 发布Offboard控制模式
	uORB::Publication<trajectory_setpoint_s> _traj_sp_pub{ORB_ID(trajectory_setpoint)};  ///< 发布轨迹设定点（速度指令）
	uORB::Publication<vehicle_command_s> _vehicle_cmd_pub{ORB_ID(vehicle_command)};  ///< 发布载具命令（模式切换等）
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};  ///< 订阅载具位置
	uORB::Publication<vehicle_attitude_setpoint_s> _att_sp_pub{ORB_ID(vehicle_attitude_setpoint)};

	// ========== 机载协同相关 ==========
	uORB::Subscription _external_mission_active_sub{ORB_ID(external_mission_active)};
	bool _external_mission_active{false};   // 机载ROS是否正在占用Offboard
	bool _allow_takeover{false};            // 当前是否允许attack_vision接管
	bool _guidance_paused{false};           // soft stop 后暂停末制导，但保留 Offboard 心跳

	// 机载/飞控协同状态输出
	struct attack_vision_status_s {
		bool lock_active;
		bool rc_offboard;
		bool external_mission_active;
		bool allow_takeover;
		int16_t pix_offset_x;
		int16_t pix_offset_y;
		uint64_t timestamp;
	};
	uORB::PublicationMulti<attack_vision_status_s> _attack_vision_status_pub{ORB_ID(attack_vision_status)};


	// ========== 模块参数 ==========
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::AAATTKVIS_EN>) _param_av_en,      ///< 模块使能开关（0=禁用，1=启用）
		(ParamInt<px4::params::AV_BAUD>) _param_av_baud,  ///< 串口波特率（默认115200）
		(ParamFloat<px4::params::AV_KP>) _param_av_kp,    ///< 速度控制增益（m/s每像素，默认0.001）
		(ParamFloat<px4::params::AV_DEAD>) _param_av_dead,  ///< 像素死区（默认5像素）
		(ParamFloat<px4::params::AV_MAX_V>) _param_av_max_v,  ///< 最大速度限制（m/s，默认1.0）
		(ParamFloat<px4::params::AV_ATT_KP>) _param_av_att_kp,  // 姿态控制增益（新增复用）
		(ParamFloat<px4::params::AV_APPROACH_V>) _param_av_approach_v,  // 最大接近速度
		(ParamInt<px4::params::AV_STRATEGY>) _param_av_strategy,  // 控制策略（1=姿态控制）
		(ParamFloat<px4::params::AV_FORWARD_V>) _param_av_forward_v
	)
};


