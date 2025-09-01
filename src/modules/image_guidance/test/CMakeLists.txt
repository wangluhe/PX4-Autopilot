include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)
px4_add_unit_gtest(
    SRC                  # 源码文件关键字
    	test_image_guidance.cpp  # 测试源码文件
)
