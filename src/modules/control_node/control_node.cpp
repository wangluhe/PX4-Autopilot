#include "control_node.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>


int ControlNode::print_status()
{
    PX4_INFO("Running");
    // TODO: print additional runtime information about the state of the module

    return 0;
}

int ControlNode::custom_command(int argc, char *argv[])
{
    /*
    if (!is_running()) {
        print_usage("not running");
        return 1;
    }

    // additional custom commands can be handled like this:
    if (!strcmp(argv[0], "do-something")) {
        get_instance()->do_something();
        return 0;
    }
     */

    return print_usage("unknown command");
}


int ControlNode::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("module",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT,
                                  1024,
                                  (px4_main_t)&run_trampoline,
                                  (char *const *)argv);

    if (_task_id < 0)
    {
        _task_id = -1;
        return -errno;
    }

    return 0;
}

ControlNode *ControlNode::instantiate(int argc, char *argv[])
{
    int example_param = 0;
    bool example_flag = false;
    bool error_flag = false;

    int myoptind = 1;
    int ch;
    const char *myoptarg = nullptr;

    // parse CLI arguments
    while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF)
    {
        switch (ch)
        {
            case 'p':
                example_param = (int)strtol(myoptarg, nullptr, 10);
                break;

            case 'f':
                example_flag = true;
                break;

            case '?':
                error_flag = true;
                break;

            default:
                PX4_WARN("unrecognized flag");
                error_flag = true;
                break;
        }
    }

    if (error_flag)
    {
        return nullptr;
    }

    ControlNode *instance = new ControlNode(example_param, example_flag);

    if (instance == nullptr)
    {
        PX4_ERR("alloc failed");
    }

    return instance;
}

ControlNode::ControlNode(int example_param, bool example_flag)
    : ModuleParams(nullptr)
{
}

void ControlNode::run()
{
    //记录起始点位置
    float begin_x;
    float begin_y;
    float begin_z;

    float_t xy_rad=1;
    float_t z_rad=1;
    float_t yaw_rad=0.1;
//进入offboard模式并解锁
    while(!should_exit())
    {
        _vehicle_status_sub.copy(&_status);
        if((_status.nav_state==vehicle_status_s::NAVIGATION_STATE_OFFBOARD)&&(_status.arming_state==vehicle_status_s::ARMING_STATE_ARMED))
        {
            PX4_INFO("in offboard and armed");
            break;
        }
        _command.target_system = _status.system_id;
        _command.target_component = _status.component_id;
        ocm.timestamp = hrt_absolute_time();
        _vehicle_local_position_sub.copy(&_vehicle_local_position);
        sp_local.x=_vehicle_local_position.x;
        sp_local.y=_vehicle_local_position.y;
        sp_local.z=_vehicle_local_position.z-5;
        sp_local.timestamp = hrt_absolute_time();
        _trajectory_setpoint_pub.publish(sp_local);
        ocm.position=true;
        ocm.timestamp = hrt_absolute_time();
        _offboard_control_mode_pub.publish(ocm);
        _command.command =vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
        _command.param1=1.0f;
        _command.param2=PX4_CUSTOM_MAIN_MODE_OFFBOARD;
        _command.timestamp = hrt_absolute_time();
        _vehicle_command_pub.publish(_command);
        usleep(10000);
        _command.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        _command.param1 = 1.0f;
        _command.timestamp = hrt_absolute_time();
        _vehicle_command_pub.publish(_command);
    }
//记录当前点的位置
    _vehicle_local_position_sub.copy(&_vehicle_local_position);
    begin_x=_vehicle_local_position.x;
    begin_y=_vehicle_local_position.y;
    begin_z=_vehicle_local_position.z;
    time_tick=hrt_absolute_time();
    //业务逻辑
    while (!should_exit())
    {
        _vehicle_local_position_sub.copy(&_vehicle_local_position);
        if(flag==0)//升至5米
        {
            flag++;
            memset(&sp_local,0,sizeof(vehicle_local_position_setpoint_s));
            sp_local.x=begin_x;
            sp_local.y=begin_y;
            sp_local.z=begin_z-5;
                    ocm.position=true;
                    ocm.velocity=false;
                    ocm.acceleration=false;
            PX4_INFO("pos 005");
        }
        if((flag==1)&&(abs(_vehicle_local_position.x-sp_local.x)<xy_rad)&&(abs(_vehicle_local_position.y-sp_local.y)<xy_rad)&&(abs(_vehicle_local_position.z-sp_local.z)<z_rad))//升至5米
        {
            if(flag2==0)
            {
                flag2++;
                time_tick=hrt_absolute_time();
            }
            if((flag2==1)&&(hrt_absolute_time()-time_tick)>1000000)
            {
                flag++;
                memset(&sp_local,0,sizeof(vehicle_local_position_setpoint_s));
                sp_local.x=begin_x+5;
                sp_local.y=begin_y;
                sp_local.z=begin_z-5;
                sp_local.vx=(float)NAN;
                sp_local.vy=(float)NAN;
                sp_local.vz=(float)NAN;
                sp_local.acceleration[0]=(float)NAN;
                sp_local.acceleration[1]=(float)NAN;
                sp_local.acceleration[2]=(float)NAN;
                ocm.position=true;
                ocm.velocity=false;
                ocm.acceleration=false;
                time_tick=hrt_absolute_time();
                PX4_INFO("pos 505");
            }

        }
        if((flag==2)&&(abs(_vehicle_local_position.x-sp_local.x)<xy_rad)&&(abs(_vehicle_local_position.y-sp_local.y)<xy_rad)&&(abs(_vehicle_local_position.z-sp_local.z)<z_rad))//升至5米
        {
            if(flag2==1)
            {
                flag2++;
                time_tick=hrt_absolute_time();
            }
            if((flag2==2)&&(hrt_absolute_time()-time_tick)>1000000)
            {
                flag++;
                memset(&sp_local,0,sizeof(vehicle_local_position_setpoint_s));
                sp_local.x=begin_x+5;
                sp_local.y=begin_y+5;
                sp_local.z=begin_z-5;
                sp_local.vx=(float)NAN;
                sp_local.vy=(float)NAN;
                sp_local.vz=(float)NAN;
                sp_local.acceleration[0]=(float)NAN;
                sp_local.acceleration[1]=(float)NAN;
                sp_local.acceleration[2]=(float)NAN;
                        ocm.position=true;
                        ocm.velocity=false;
                        ocm.acceleration=false;
                time_tick=hrt_absolute_time();
                PX4_INFO("pos 555");
            }

        }
        if(flag==3&&(abs(_vehicle_local_position.x-sp_local.x)<xy_rad)&&(abs(_vehicle_local_position.y-sp_local.y)<xy_rad)&&(abs(_vehicle_local_position.z-sp_local.z)<z_rad))
        {
            if(flag2==2)
            {
                flag2++;
                time_tick=hrt_absolute_time();
            }
            if((flag2==3)&&(hrt_absolute_time()-time_tick)>1000000)
            {
            flag++;
            memset(&sp_local,0,sizeof(vehicle_local_position_setpoint_s));
            sp_local.x=begin_x+5;
            sp_local.y=begin_y+5;
            sp_local.z=begin_z-5;
            sp_local.vx=(float)NAN;
            sp_local.vy=(float)NAN;
            sp_local.vz=(float)NAN;
            sp_local.acceleration[0]=(float)NAN;
            sp_local.acceleration[1]=(float)NAN;
            sp_local.acceleration[2]=(float)NAN;
            sp_local.yaw=1;
            sp_local.yawspeed=(float)NAN;
            ocm.position=true;
            ocm.velocity=false;
            ocm.acceleration=false;
            PX4_INFO("yaw");
            }
        }
        if(flag==4&&(abs(_vehicle_local_position.heading-sp_local.yaw)<yaw_rad))
        {
            if(flag2==3)
            {
                flag2++;
                time_tick=hrt_absolute_time();
            }
            if((flag2==4)&&(hrt_absolute_time()-time_tick)>1000000)
            {
            flag++;
            memset(&sp_local,0,sizeof(vehicle_local_position_setpoint_s));
            sp_local.x=begin_x+5;
            sp_local.y=begin_y+5;
            sp_local.z=begin_z-5;
            sp_local.vx=(float)NAN;
            sp_local.vy=(float)NAN;
            sp_local.vz=(float)NAN;
            sp_local.acceleration[0]=(float)NAN;
            sp_local.acceleration[1]=(float)NAN;
            sp_local.acceleration[2]=(float)NAN;
            sp_local.yaw=(float)NAN;
            sp_local.yawspeed=1;
            ocm.position=true;
            ocm.velocity=false;
            ocm.acceleration=false;
            PX4_INFO("yawspeed");
            }
        }
        if(flag==5)
        {
            if(flag2==4)
            {
                flag2++;
                time_tick=hrt_absolute_time();
            }
            if((flag2==5)&&(hrt_absolute_time()-time_tick)>5000000)
            {
            flag++;
            memset(&sp_local,0,sizeof(vehicle_local_position_setpoint_s));
            sp_local.x=(float)NAN;
            sp_local.y=(float)NAN;
            sp_local.z=begin_z-5;
            sp_local.vx=1;
            sp_local.vy=0;
            sp_local.vz=(float)NAN;
            sp_local.acceleration[0]=(float)NAN;
            sp_local.acceleration[1]=(float)NAN;
            sp_local.acceleration[2]=(float)NAN;
            sp_local.yaw=(float)NAN;
            sp_local.yawspeed=(float)NAN;
            ocm.position=true;
            ocm.velocity=true;
            ocm.acceleration=false;
            PX4_INFO("velocity");
            }
        }
        if(flag==6)
        {
            if(flag2==5)
            {
                flag2++;
                time_tick=hrt_absolute_time();
            }
            if((flag2==6)&&(hrt_absolute_time()-time_tick)>5000000)
            {
            flag++;
            memset(&sp_local,0,sizeof(vehicle_local_position_setpoint_s));
            sp_local.x=(float)NAN;
            sp_local.y=(float)NAN;
            sp_local.z=begin_z-5;
            sp_local.vx=(float)NAN;
            sp_local.vy=(float)NAN;
            sp_local.vz=(float)NAN;
            sp_local.acceleration[0]=0;
            sp_local.acceleration[1]=1;
            sp_local.acceleration[2]=(float)NAN;
            sp_local.yaw=(float)NAN;
            sp_local.yawspeed=(float)NAN;
            ocm.position=true;
            ocm.velocity=false;
            ocm.acceleration=true;
            PX4_INFO("acceleration");
            }
        }

        if(flag==7)
        {
            if(flag2==6)
            {
                flag2++;
                time_tick=hrt_absolute_time();
            }
            if((flag2==7)&&(hrt_absolute_time()-time_tick)>5000000)
            {
            flag++;
            _command.command =vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
            _command.param1=1.0f;
            _command.param2=PX4_CUSTOM_MAIN_MODE_AUTO;
            _command.param3=PX4_CUSTOM_SUB_MODE_AUTO_RTL;
            _command.timestamp = hrt_absolute_time();
            _vehicle_command_pub.publish(_command);
            PX4_INFO("return");
            }
        }


if(ocm.position||ocm.velocity||ocm.acceleration)
{
        ocm.timestamp = hrt_absolute_time();
        _offboard_control_mode_pub.publish(ocm);
        _vehicle_status_sub.copy(&_status);
if(_status.nav_state==vehicle_status_s::NAVIGATION_STATE_OFFBOARD)
{
        sp_local.timestamp = hrt_absolute_time();
        _trajectory_setpoint_pub.publish(sp_local);
//        PX4_INFO("sp_local.x=%lf\n",(double)sp_local.x);
//        PX4_INFO("sp_local.y=%lf\n",(double)sp_local.y);
//        PX4_INFO("sp_local.z=%lf\n",(double)sp_local.z);
//        PX4_INFO("sp_local.vx=%lf\n",(double)sp_local.vx);
//        PX4_INFO("sp_local.vy=%lf\n",(double)sp_local.vy);
//        PX4_INFO("sp_local.vz=%lf\n",(double)sp_local.vz);
}
}
        usleep(100000);

        parameters_update();
    }
}

void ControlNode::parameters_update(bool force)
{
    // check for parameter updates
    if (_parameter_update_sub.updated() || force)
    {
        // clear update
        parameter_update_s update;
        _parameter_update_sub.copy(&update);

        // update parameters from storage
        updateParams();
    }
}

int ControlNode::print_usage(const char *reason)
{
    if (reason)
    {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int control_node_main(int argc, char *argv[])
{
	return ControlNode::main(argc, argv);
}
