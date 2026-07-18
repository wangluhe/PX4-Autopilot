#pragma once

#include <cstdint>

#include <matrix/matrix/math.hpp>

namespace attack_vision_protocol
{

static constexpr int FRAME_LEN{64};
static constexpr uint8_t FRAME_HEAD_0{0xFC};
static constexpr uint8_t FRAME_HEAD_1{0x2C};
static constexpr uint8_t FRAME_TAIL{0xF0};

struct ParsedFrameData {
	bool lock_active{false};
	int16_t pix_offset_x{0};
	int16_t pix_offset_y{0};
	int16_t roll_deg_100{0};
	int16_t pitch_deg_100{0};
	int16_t yaw_deg_100{0};
	float gimbal_roll_rad{0.0f};
	float gimbal_pitch_rad{0.0f};
	float gimbal_yaw_rad{0.0f};
};

bool validate_frame(const uint8_t *frame);

bool parse_frame(const uint8_t *frame, bool gimbal_roll_inv, bool gimbal_pitch_inv,
		bool gimbal_yaw_inv, ParsedFrameData &out);

} // namespace attack_vision_protocol
