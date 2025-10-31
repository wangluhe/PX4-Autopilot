#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/time.h>
#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>

#include <termios.h>
#include <poll.h>

class AttackVision : public ModuleBase<AttackVision>, public ModuleParams, public px4::WorkItem
{
public:
	AttackVision();
	~AttackVision() override;

	// ModuleBase API
	static int task_spawn(int argc, char *argv[]);
	static AttackVision *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	int print_status() override;
	void Run() override;

private:
	// UART
	int _fd{-1};
	bool configure_uart(int baudrate);
	bool open_uart();

	// Frame parsing
	static constexpr int FRAME_LEN{64};
	static constexpr uint8_t FRAME_HEAD_0{0xFC};
	static constexpr uint8_t FRAME_HEAD_1{0x2C};
	static constexpr uint8_t FRAME_TAIL{0xF0};
	uint8_t _buf[FRAME_LEN]{};
	int _buf_len{0};
	uint64_t _last_frame_time_us{0};
	bool try_read_frame();
	bool validate_frame(const uint8_t *frame);

	// Decoded fields of interest
	bool _lock_active{false};
	int16_t _pix_offset_x{0}; // bytes 59-60 azimuth
	int16_t _pix_offset_y{0}; // bytes 61-62 pitch

	// Control
	void handle_guidance();
	void publish_offboard_velocity(float vx, float vy, float vz, float yaw_rate);
	bool switch_to_offboard();
	void switch_to_hold();

	// uORB
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Publication<offboard_control_mode_s> _offboard_ctrl_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<trajectory_setpoint_s> _traj_sp_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<vehicle_command_s> _vehicle_cmd_pub{ORB_ID(vehicle_command)};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::AV_EN>) _param_av_en,
		(ParamInt<px4::params::AV_BAUD>) _param_av_baud,
		(ParamFloat<px4::params::AV_KP>) _param_av_kp,
		(ParamFloat<px4::params::AV_DEAD>) _param_av_dead,
		(ParamFloat<px4::params::AV_MAX_V>) _param_av_max_v
	)
};


