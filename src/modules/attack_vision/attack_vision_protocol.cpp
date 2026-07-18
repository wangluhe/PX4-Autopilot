#include "attack_vision_protocol.hpp"

#include <cmath>

#include <px4_platform_common/px4_config.h>

namespace attack_vision_protocol
{

bool validate_frame(const uint8_t *frame)
{
	if (!frame) {
		return false;
	}

	if (frame[0] != FRAME_HEAD_0 || frame[1] != FRAME_HEAD_1) {
		return false;
	}

	if (frame[63] != FRAME_TAIL) {
		return false;
	}

	uint8_t xorv = 0;
	for (int i = 2; i <= 61; i++) {
		xorv ^= frame[i];
	}

	return xorv == frame[62];
}

bool parse_frame(const uint8_t *frame, bool gimbal_roll_inv, bool gimbal_pitch_inv,
		bool gimbal_yaw_inv, ParsedFrameData &out)
{
	if (!validate_frame(frame)) {
		return false;
	}

	const uint16_t status_5_6 = static_cast<uint16_t>(frame[4]) |
		(static_cast<uint16_t>(frame[5]) << 8);
	const uint8_t servo_state = frame[8];
	const bool bit9 = (status_5_6 & (1 << 9)) != 0;
	const bool bit10 = (status_5_6 & (1 << 10)) != 0;
	const int lock_state = ((bit10 ? 1 : 0) << 1) | (bit9 ? 1 : 0);
	const bool locking = (lock_state == 1) || (lock_state == 2);

	out.lock_active = locking && (servo_state == 0x07);
	out.pix_offset_x = static_cast<int16_t>(static_cast<uint16_t>(frame[58]) |
		(static_cast<uint16_t>(frame[59]) << 8));
	out.pix_offset_y = static_cast<int16_t>(static_cast<uint16_t>(frame[60]) |
		(static_cast<uint16_t>(frame[61]) << 8));
	out.roll_deg_100 = static_cast<int16_t>(static_cast<uint16_t>(frame[13]) |
		(static_cast<uint16_t>(frame[14]) << 8));
	out.pitch_deg_100 = static_cast<int16_t>(static_cast<uint16_t>(frame[11]) |
		(static_cast<uint16_t>(frame[12]) << 8));
	out.yaw_deg_100 = static_cast<int16_t>(static_cast<uint16_t>(frame[9]) |
		(static_cast<uint16_t>(frame[10]) << 8));

	const float roll_rad = (out.roll_deg_100 / 100.0f) * M_PI_F / 180.0f;
	const float pitch_rad = (out.pitch_deg_100 / 100.0f) * M_PI_F / 180.0f;
	const float yaw_rad = (out.yaw_deg_100 / 100.0f) * M_PI_F / 180.0f;

	out.gimbal_roll_rad = gimbal_roll_inv ? -roll_rad : roll_rad;
	out.gimbal_pitch_rad = gimbal_pitch_inv ? -pitch_rad : pitch_rad;
	out.gimbal_yaw_rad = gimbal_yaw_inv ? -yaw_rad : yaw_rad;

	return true;
}

} // namespace attack_vision_protocol
