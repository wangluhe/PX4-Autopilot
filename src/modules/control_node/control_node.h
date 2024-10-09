#pragma once
#include "commander/px4_custom_mode.h"
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>

using namespace time_literals;

extern "C" __EXPORT int control_node_main(int argc, char *argv[]);


class ControlNode : public ModuleBase<ControlNode>, public ModuleParams
{
public:
	ControlNode(int example_param, bool example_flag);

	virtual ~ControlNode() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static ControlNode *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);


	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
	)

    uint64_t time_tick=hrt_absolute_time();
    int flag=0,flag2=0;
    vehicle_local_position_s _vehicle_local_position;
    vehicle_status_s _status;
    vehicle_command_s _command = {};
    offboard_control_mode_s ocm{};
    position_setpoint_triplet_s _pos_sp_triplet{};
    vehicle_command_ack_s _ack{};
    vehicle_local_position_setpoint_s sp_local{};
    vehicle_attitude_setpoint_s attitude_setpoint{};
	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
        uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
        uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

       // Publications
       uORB::Publication<offboard_control_mode_s>		_offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
       uORB::Publication<vehicle_command_s>		_vehicle_command_pub{ORB_ID(vehicle_command)};
       uORB::Publication<position_setpoint_triplet_s>		_position_setpoint_triplet_pub{ORB_ID(position_setpoint_triplet)};
       uORB::Publication<vehicle_local_position_setpoint_s>	_trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
};
