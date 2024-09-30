#include "WorkItemTest.hpp"

WorkItemTest::WorkItemTest() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

WorkItemTest::~WorkItemTest()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool WorkItemTest::init()
{
ScheduleOnInterval(100000_us);
	return true;
}

void WorkItemTest::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(¶m_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}
	double param_test1=_param_test1.get();
        int param_test2=_param_test2.get();
        PX4_INFO("param_test1=%lf param_test2=%d\n",param_test1,param_test2);

	PX4_INFO("test workitem");
	_log_test.timestamp=hrt_absolute_time();
	_log_test.seq++;
	 PX4_INFO("_log_test.seq=%d\n",_log_test.seq);
	_log_test_pub.publish(_log_test);

	perf_end(_loop_perf);
}

int WorkItemTest::task_spawn(int argc, char *argv[])
{
	WorkItemTest *instance = new WorkItemTest();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int WorkItemTest::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int WorkItemTest::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int WorkItemTest::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("work_item_example", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int work_item_test_main(int argc, char *argv[])
{
	return WorkItemTest::main(argc, argv);
}
