#include "image_guidance.h"
#include <gtest/gtest.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <px4_platform_common/log.h>

class ImageGuidanceTest : public ::testing::Test {
protected:
    ImageGuidance *guidance;

    void SetUp() override {
        guidance = new ImageGuidance();
    }

    void TearDown() override {
        delete guidance;
    }
};

TEST_F(ImageGuidanceTest, ConstructorInitializesCorrectly) {
    EXPECT_NE(guidance, nullptr);
}

TEST_F(ImageGuidanceTest, InitFailsWithInvalidUART) {
    // Mock invalid UART path
    const char *invalid_path = "/dev/invalid";
    EXPECT_EQ(guidance->init(), -1);
}

/**
 * @brief 测试解析帧数据时校验和的有效性。
 *
 * 该测试用例验证了当输入帧数据包含有效的校验和时，`parse_frame` 方法能够正确解析帧数据。
 * 测试数据包括同步头、有效载荷和帧尾，确保每个字节都能被正确解析。
 */
TEST_F(ImageGuidanceTest, ParseFrameValidChecksum) {
    uint8_t valid_frame[] = {SYNC_HEADER1, SYNC_HEADER2, 0x01, 0x02, 0x03, FRAME_TAIL};
    for (size_t i = 0; i < sizeof(valid_frame); ++i) {
        EXPECT_TRUE(guidance->parse_frame(valid_frame[i]));
    }
}

TEST_F(ImageGuidanceTest, ParseFrameInvalidChecksum) {
    uint8_t invalid_frame[] = {SYNC_HEADER1, SYNC_HEADER2, 0x01, 0x02, 0x04, FRAME_TAIL};
    for (size_t i = 0; i < sizeof(invalid_frame); ++i) {
        EXPECT_FALSE(guidance->parse_frame(invalid_frame[i]));
    }
}

TEST_F(ImageGuidanceTest, CalculateControlWithTrackingActive) {
    guidance->_tracking_active = true;
    guidance->_azimuth_offset = 100;
    guidance->_pitch_offset = 50;
    guidance->calculate_control();
    EXPECT_NEAR(guidance->_control_setpoint.roll, 0.1f, 0.001f);
    EXPECT_NEAR(guidance->_control_setpoint.pitch, 0.05f, 0.001f);
}

TEST_F(ImageGuidanceTest, CalculateControlWithTrackingInactive) {
    guidance->_tracking_active = false;
    guidance->calculate_control();
    EXPECT_TRUE(isnan(guidance->_control_setpoint.roll));
    EXPECT_TRUE(isnan(guidance->_control_setpoint.pitch));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
