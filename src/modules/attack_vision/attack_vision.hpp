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
 * 硬件连接：吊舱串口连接飞控TELEM2口（/dev/ttyS5）
 */

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <matrix/matrix/math.hpp>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include "attack_vision_protocol.hpp"
#include "attack_vision_guidance.hpp"
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
#include <uORB/topics/parameter_update.h>

/**
 * @class AttackVision
 * @brief 图像末制导模块主类
 *
 * 继承自ModuleBase、ModuleParams和ScheduledWorkItem，实现PX4模块标准接口
 */
class AttackVision : public ModuleBase<AttackVision>, public ModuleParams, public px4::ScheduledWorkItem
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
	bool open_uart();  ///< 打开串口设备（TELEM2，实机/dev/ttyS5）

	// ========== 帧解析相关 ==========
	static constexpr int FRAME_LEN{attack_vision_protocol::FRAME_LEN};  ///< 吊舱反馈帧固定长度：64字节
	static constexpr uint8_t FRAME_HEAD_0{attack_vision_protocol::FRAME_HEAD_0};  ///< 帧头字节0
	static constexpr uint8_t FRAME_HEAD_1{attack_vision_protocol::FRAME_HEAD_1};  ///< 帧头字节1
	static constexpr uint8_t FRAME_TAIL{attack_vision_protocol::FRAME_TAIL};  ///< 帧尾字节
	uint8_t _buf[FRAME_LEN]{};  ///< 接收缓冲区
	int _buf_len{0};  ///< 当前缓冲区数据长度
	uint64_t _last_frame_time_us{0};  ///< 最后一次有效帧的时间戳（用于超时检测）
	uint32_t _frame_sequence{0};  ///< 通过校验的有效吊舱帧序号
	static constexpr uint64_t FRAME_TIMEOUT_US = 200000;  ///< 200ms无有效帧视为失效
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
	matrix::Vector3f _last_raw_los_gimbal{}; // 最近一次未经滤波的吊舱坐标系LOS
	matrix::Vector3f _last_los_gimbal{};      // 最近一次像素脱靶量转换出的吊舱FRD视线
	matrix::Vector3f _last_target_vec_ned{};  // 最近一次转换到NED的目标视线
	bool _last_los_gimbal_valid{false};
	bool _last_raw_los_gimbal_valid{false};
	bool _last_target_vec_ned_valid{false};
	GuidanceCommand _last_guidance_command{};
	hrt_abstime _last_guidance_command_time_us{0};
	AlphaFilter<matrix::Vector3f> _los_filter{}; ///< 吊舱坐标系下的LOS一阶低通
	bool _los_filter_initialized{false};
	uint64_t _los_filter_sample_time_us{0};

	// ========== 状态机相关 ==========
	enum class ModuleState {
		HOLD,                   ///< 悬停模式
		SWITCHING_TO_OFFBOARD,  ///< 正在切换到Offboard模式
		OFFBOARD                ///< Offboard模式激活
	};
	ModuleState _module_state{ModuleState::HOLD};  ///< 当前模块状态

	// ========== 控制逻辑相关 ==========

	float _forward_velocity{1.0f};  ///< 恒定前向速度
	hrt_abstime _switch_start_time{0};  ///< 模式切换开始时间戳
	hrt_abstime _last_cmd_publish_time{0};  ///< 上次命令发布的时间戳（用于频率限制）
	hrt_abstime _last_offboard_exit_time{0};  ///< 最近一次检测到人工退出Offboard的时间戳
	hrt_abstime _last_status_time{0};  ///< 最近一次状态日志时间戳
	hrt_abstime _last_control_time{0};  ///< 最近一次控制输出时间戳
	hrt_abstime _last_drone_status_time{0};  ///< 最近一次无人机状态日志时间戳
	hrt_abstime _last_frame_log_time{0};  ///< 最近一次吊舱帧日志时间戳
	bool _manual_offboard_exit_latched{false};  ///< QGC/外部切出Offboard后禁止自动拉回
	bool _was_vehicle_in_offboard{false};  ///< 用于检测Offboard退出边沿
	bool _run_initialized{false};  ///< Run()首次调度时完成一次性初始化
	static constexpr uint64_t MIN_CMD_INTERVAL_US = 500000;  ///< 命令发布最小间隔（500ms）
	void handle_guidance();  ///< 处理制导逻辑：根据像素偏差计算速度指令
	bool check_guidance_ready();
	bool read_vehicle_guidance_state(VehicleGuidanceState &state);
	bool get_gimbal_ned_pose(float vehicle_yaw, GimbalNedPose &pose);
	bool build_vision_target(VisionTarget &target);
	bool build_camera_model(CameraModel &camera);
	bool build_target_los_gimbal(const VisionTarget &target, const CameraModel &camera, matrix::Vector3f &los_gimbal);
	void reset_los_filter();
	bool update_los_filter(const matrix::Vector3f &raw_los_gimbal, matrix::Vector3f &filtered_los_gimbal);
	bool build_guidance_command(const VehicleGuidanceState &veh, const GimbalNedPose &gimbal,
					const VisionTarget &target, const CameraModel &camera, GuidanceCommand &cmd);
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
	void update_external_mission_state(uint64_t now_us);

	// ========== uORB话题订阅和发布 ==========
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};  ///< 订阅载具状态
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};  // 订阅无人机当前姿态
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};  ///< 订阅参数更新，支持运行中调参
	uORB::Publication<offboard_control_mode_s> _offboard_ctrl_pub{ORB_ID(offboard_control_mode)};  ///< 发布Offboard控制模式
	uORB::Publication<trajectory_setpoint_s> _traj_sp_pub{ORB_ID(trajectory_setpoint)};  ///< 发布轨迹设定点（速度指令）
	uORB::Publication<vehicle_command_s> _vehicle_cmd_pub{ORB_ID(vehicle_command)};  ///< 发布载具命令（模式切换等）
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};  ///< 订阅载具位置
	uORB::Publication<vehicle_attitude_setpoint_s> _att_sp_pub{ORB_ID(vehicle_attitude_setpoint)};

	// ========== 机载协同相关 ==========
	static constexpr int AV_EXT_MODE_STANDALONE = 0;
	static constexpr int AV_EXT_MODE_COORDINATED = 1;
	static constexpr int AV_EXT_MODE_AUTO = 2;
	static constexpr uint64_t EXTERNAL_MISSION_TIMEOUT_US = 1000000;
	uORB::Subscription _external_mission_active_sub{ORB_ID(external_mission_active)};
	bool _external_mission_active{false};   // 生效后的机载ROS是否正在占用Offboard
	bool _external_mission_active_raw{false};   // uORB原始external_mission_active输入
	bool _external_mission_seen{false};   // 是否接收过上位机协同状态
	bool _coordinated_mode_active{false};   // 当前是否按上位机协同模式运行
	hrt_abstime _last_external_mission_time_us{0};   // 最近一次收到上位机协同状态的时间
	bool _allow_takeover{false};            // 当前是否允许attack_vision接管
	bool _guidance_paused{false};           // soft stop 后暂停末制导，但保留 Offboard 心跳

	// 机载/飞控协同状态输出
	uORB::Publication<attack_vision_status_s> _attack_vision_status_pub{ORB_ID(attack_vision_status)};


	// ========== 模块参数 ==========
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::AAATTKVIS_EN>) _param_av_en,      ///< 模块使能开关（0=禁用，1=启用）
		(ParamInt<px4::params::AV_BAUD>) _param_av_baud,  ///< 串口波特率（默认115200）
		(ParamFloat<px4::params::AV_KP>) _param_av_kp,    ///< 旧像素控制参数，当前制导链路保留兼容
		(ParamFloat<px4::params::AV_DEAD>) _param_av_dead,  ///< 旧像素控制参数，当前制导链路保留兼容
		(ParamFloat<px4::params::AV_MAX_V>) _param_av_max_v,  ///< 旧像素控制参数，当前制导链路保留兼容
		(ParamFloat<px4::params::AV_ATT_KP>) _param_av_att_kp,  // 姿态控制增益（新增复用）
		(ParamFloat<px4::params::AV_APPROACH_V>) _param_av_approach_v,  // 最大接近速度
		(ParamInt<px4::params::AV_STRATEGY>) _param_av_strategy,  // 旧策略选择参数，当前制导链路保留兼容
		(ParamFloat<px4::params::AV_FORWARD_V>) _param_av_forward_v,
		(ParamInt<px4::params::AV_EXT_MODE>) _param_av_ext_mode,
		(ParamInt<px4::params::AV_CAM_W>) _param_av_cam_w,
		(ParamInt<px4::params::AV_CAM_H>) _param_av_cam_h,
		(ParamFloat<px4::params::AV_FOV_H>) _param_av_fov_h,
		(ParamFloat<px4::params::AV_FOV_V>) _param_av_fov_v,
		(ParamInt<px4::params::AV_PIX_X_INV>) _param_av_pix_x_inv,
		(ParamInt<px4::params::AV_PIX_Y_INV>) _param_av_pix_y_inv,
		(ParamInt<px4::params::AV_GMB_YAW_INV>) _param_av_gmb_yaw_inv,
		(ParamInt<px4::params::AV_GMB_PIT_INV>) _param_av_gmb_pit_inv,
		(ParamInt<px4::params::AV_GMB_ROLL_INV>) _param_av_gmb_roll_inv,
		(ParamFloat<px4::params::AV_MNT_YAW>) _param_av_mnt_yaw,
		(ParamFloat<px4::params::AV_MAX_VZ>) _param_av_max_vz,
		(ParamFloat<px4::params::AV_LOS_TAU>) _param_av_los_tau,
		(ParamInt<px4::params::AV_DN_SHAPE_EN>) _param_av_dn_shape_en,
		(ParamFloat<px4::params::AV_LOCAL_H_STOP>) _param_av_local_h_stop,
		(ParamFloat<px4::params::AV_LOCAL_H_FULL>) _param_av_local_h_full,
		(ParamFloat<px4::params::AV_PITCH_STOP>) _param_av_pitch_stop,
		(ParamFloat<px4::params::AV_PITCH_FULL>) _param_av_pitch_full,
		(ParamInt<px4::params::AV_SIM_PIX_EN>) _param_av_sim_pix_en,
		(ParamInt<px4::params::AV_SIM_PIX_X>) _param_av_sim_pix_x,
		(ParamInt<px4::params::AV_SIM_PIX_Y>) _param_av_sim_pix_y,
		(ParamInt<px4::params::AV_SIM_GMB_EN>) _param_av_sim_gmb_en,
		(ParamInt<px4::params::AV_SIM_GMB_ROLL>) _param_av_sim_gmb_roll,
		(ParamInt<px4::params::AV_SIM_GMB_PIT>) _param_av_sim_gmb_pit,
		(ParamInt<px4::params::AV_SIM_GMB_YAW>) _param_av_sim_gmb_yaw
	)
};
