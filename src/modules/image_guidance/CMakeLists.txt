px4_add_module(
    MODULE modules__image_guidance
    MAIN image_guidance
    SRCS
        image_guidance.cpp
    INCLUDES
        .
    DEPENDS
        px4_platform
        drivers_device
        uORB
        perf
        parameters
)

# Generate uORB topic headers
add_dependencies(${MODULE_NAME} uorb_messages)
