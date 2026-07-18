#pragma once

#include <cstdint>

#include <matrix/matrix/math.hpp>

struct VehicleGuidanceState {
	matrix::Quatf q_veh_ned{};
	float veh_roll{0.0f};
	float veh_pitch{0.0f};
	float veh_yaw{0.0f};
	bool valid{false};
};

struct GimbalNedPose {
	matrix::Quatf q_gimbal_ned{};
	float gimbal_roll_ned{0.0f};
	float gimbal_pitch_ned{0.0f};
	float gimbal_yaw_ned{0.0f};
	bool valid{false};
};

struct VisionTarget {
	int16_t pix_offset_x{0};
	int16_t pix_offset_y{0};
	bool valid{false};
};

struct CameraModel {
	float width_px{1920.0f};
	float height_px{1080.0f};
	float fov_h_rad{0.0f};
	float fov_v_rad{0.0f};
	bool pixel_y_positive_up{true};
	bool valid{false};
};

struct GuidanceCommand {
	float target_roll{0.0f};
	float target_pitch{0.0f};
	float target_yaw{0.0f};
	float vx_ned{0.0f};
	float vy_ned{0.0f};
	float vz_ned{0.0f};
	float target_yaw_rate{0.0f};
	bool valid{false};
};

namespace attack_vision_guidance
{

bool build_vision_target(int16_t pix_offset_x, int16_t pix_offset_y, bool lock_active,
			VisionTarget &target);

bool build_camera_model(int cam_w, int cam_h, float fov_h_deg, float fov_v_deg,
			bool pixel_y_positive_up, CameraModel &camera);

bool build_target_los_gimbal(const VisionTarget &target, const CameraModel &camera,
			bool pix_x_inv, matrix::Vector3f &los_gimbal);

bool compose_gimbal_ned_pose(const matrix::Quatf &q_veh_ned, float gimbal_roll,
			float gimbal_pitch, float gimbal_yaw, GimbalNedPose &pose);

bool build_guidance_command(const VehicleGuidanceState &veh, const matrix::Vector3f &target_vec_ned,
			float max_forward_v, float max_vz, GuidanceCommand &cmd);

} // namespace attack_vision_guidance
