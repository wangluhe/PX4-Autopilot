px4_add_module(
	MODULE modules__image_guidance
	MAIN image_guidance
	SRCS
		image_guidance.cpp
	INCLUDES
		.
	DEPENDS
		px4_platform
		drivers__device
		uORB
		perf
)

# Generate uORB topic headers
# add_dependencies(modules__image_guidance uorb_messages)
add_dependencies(modules__image_guidance uorb_msgs)
# add_dependencies(${MODULE_NAME} uorb_messages)
