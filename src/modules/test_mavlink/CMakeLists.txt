px4_add_module(
	MODULE modules__test_mavlink
	MAIN test_mavlink
	COMPILE_FLAGS
		#-DDEBUG_BUILD   # uncomment for PX4_DEBUG output
		#-O0             # uncomment when debugging
	SRCS
		test_mavlink.cpp
	DEPENDS
		px4_work_queue
	)
