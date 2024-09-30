#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/test_mavlink_tx.h>
#include <uORB/topics/test_mavlink_rx.h>
using namespace time_literals;

extern "C" __EXPORT int test_mavlink_main(int argc, char *argv[]);


class test_mavlink : public ModuleBase<test_mavlink>, public ModuleParams
{
public:
    test_mavlink(int example_param, bool example_flag);

    virtual ~test_mavlink() = default;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static test_mavlink *instantiate(int argc, char *argv[]);

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
    test_mavlink_tx_s _test_mavlink_tx;
    test_mavlink_rx_s _test_mavlink_rx;
    // Subscriptions
    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
    uORB::Subscription test_mavlink_rx_sub{ORB_ID(test_mavlink_rx)};
    uORB::Publication<test_mavlink_tx_s>	test_mavlink_tx_pub{ORB_ID(test_mavlink_tx)};
};
