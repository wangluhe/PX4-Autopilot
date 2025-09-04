include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)
px4_add_unit_gtest(
    SRC
    	test_image_guidance.cpp
    LINKLIBS
        modules__image_guidance
)
