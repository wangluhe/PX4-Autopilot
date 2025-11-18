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
#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>

#include <termios.h>
#include <poll.h>
#include <uORB/Subscription.hpp>

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
	int print_status() override;
	void Run() override;

private:
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

	// ========== 状态机相关 ==========
	enum class ModuleState {
		HOLD,                   ///< 悬停模式
		SWITCHING_TO_OFFBOARD,  ///< 正在切换到Offboard模式
		OFFBOARD                ///< Offboard模式激活
	};
	ModuleState _module_state{ModuleState::HOLD};  ///< 当前模块状态

	// ========== 控制逻辑相关 ==========
	hrt_abstime _switch_start_time{0};  ///< 模式切换开始时间戳
	hrt_abstime _last_cmd_publish_time{0};  ///< 上次命令发布的时间戳（用于频率限制）
	static constexpr uint64_t MIN_CMD_INTERVAL_US = 500000;  ///< 命令发布最小间隔（500ms）
	void handle_guidance();  ///< 处理制导逻辑：根据像素偏差计算速度指令
	void publish_offboard_velocity(float vx, float vy, float vz, float yaw_rate);  ///< 发布Offboard速度设定值
	bool switch_to_offboard();  ///< 切换到Offboard模式（如果尚未切换）
	void switch_to_hold();  ///< 切换到悬停模式（Loiter）
	void parse_frame_data();
	void close_uart();

	// ========== uORB话题订阅和发布 ==========
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};  ///< 订阅载具状态
	uORB::Publication<offboard_control_mode_s> _offboard_ctrl_pub{ORB_ID(offboard_control_mode)};  ///< 发布Offboard控制模式
	uORB::Publication<trajectory_setpoint_s> _traj_sp_pub{ORB_ID(trajectory_setpoint)};  ///< 发布轨迹设定点（速度指令）
	uORB::Publication<vehicle_command_s> _vehicle_cmd_pub{ORB_ID(vehicle_command)};  ///< 发布载具命令（模式切换等）

	// ========== 模块参数 ==========
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::AAATTKVIS_EN>) _param_av_en,      ///< 模块使能开关（0=禁用，1=启用）
		(ParamInt<px4::params::AV_BAUD>) _param_av_baud,  ///< 串口波特率（默认115200）
		(ParamFloat<px4::params::AV_KP>) _param_av_kp,    ///< 速度控制增益（m/s每像素，默认0.001）
		(ParamFloat<px4::params::AV_DEAD>) _param_av_dead,  ///< 像素死区（默认5像素）
		(ParamFloat<px4::params::AV_MAX_V>) _param_av_max_v  ///< 最大速度限制（m/s，默认1.0）
	)
};


