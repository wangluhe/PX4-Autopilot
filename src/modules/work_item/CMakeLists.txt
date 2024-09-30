px4_add_module(
	MODULE modules__work_item
	MAIN work_item_test
	COMPILE_FLAGS
		#-DDEBUG_BUILD   # uncomment for PX4_DEBUG output
		#-O0             # uncomment when debugging
	SRCS
		WorkItemTest.cpp
		WorkItemTest.hpp
	DEPENDS
		px4_work_queue
	)
