#include "attack_vision_guidance.hpp"

#include <cmath>

#include <px4_platform_common/px4_config.h>
#include <lib/mathlib/math/Limits.hpp>

namespace attack_vision_guidance
{

static float smoothstep(float edge0, float edge1, float value)
{
	const float t = math::constrain((value - edge0) / (edge1 - edge0), 0.0f, 1.0f);
	return t * t * (3.0f - 2.0f * t);
}

bool build_vision_target(int16_t pix_offset_x, int16_t pix_offset_y, bool lock_active,
			VisionTarget &target)
{
	target.pix_offset_x = pix_offset_x;
	target.pix_offset_y = pix_offset_y;
	target.valid = lock_active;
	return target.valid;
}

bool build_camera_model(int cam_w, int cam_h, float fov_h_deg, float fov_v_deg,
			bool pixel_y_positive_up, CameraModel &camera)
{
	if (cam_w <= 0 || cam_h <= 0 ||
		!PX4_ISFINITE(fov_h_deg) || !PX4_ISFINITE(fov_v_deg) ||
		fov_h_deg <= 0.0f || fov_h_deg >= 179.0f ||
		fov_v_deg <= 0.0f || fov_v_deg >= 179.0f) {
		return false;
	}

	camera.width_px = static_cast<float>(cam_w);
	camera.height_px = static_cast<float>(cam_h);
	camera.fov_h_rad = fov_h_deg * M_PI_F / 180.0f;
	camera.fov_v_rad = fov_v_deg * M_PI_F / 180.0f;
	camera.pixel_y_positive_up = pixel_y_positive_up;
	camera.valid = true;
	return true;
}

bool build_target_los_gimbal(const VisionTarget &target, const CameraModel &camera,
			bool pix_x_inv, float image_roll_rad, matrix::Vector3f &los_gimbal)
{
	if (!target.valid || !camera.valid) {
		return false;
	}

	const float half_w = camera.width_px * 0.5f;
	const float half_h = camera.height_px * 0.5f;
	if (half_w <= 0.0f || half_h <= 0.0f) {
		return false;
	}

	const float pix_x = math::constrain(static_cast<float>(target.pix_offset_x), -half_w, half_w);
	const float pix_y = math::constrain(static_cast<float>(target.pix_offset_y), -half_h, half_h);
	const float x_tan = (pix_x / half_w) * tanf(camera.fov_h_rad * 0.5f);
	const float y_tan = (pix_y / half_h) * tanf(camera.fov_v_rad * 0.5f);
	const float right_tan = pix_x_inv ? -x_tan : x_tan;
	const float down_tan = camera.pixel_y_positive_up ? -y_tan : y_tan;
	const float cos_roll = cosf(image_roll_rad);
	const float sin_roll = sinf(image_roll_rad);
	const float right_corr = right_tan * cos_roll - down_tan * sin_roll;
	const float down_corr = right_tan * sin_roll + down_tan * cos_roll;

	los_gimbal = matrix::Vector3f(1.0f, right_corr, down_corr);
	if (los_gimbal.norm() <= 1e-3f) {
		return false;
	}

	los_gimbal.normalize();
	return true;
}

bool compose_gimbal_ned_pose(float vehicle_yaw, float mount_yaw,
			float gimbal_pitch, float gimbal_yaw, GimbalNedPose &pose)
{
	if (!PX4_ISFINITE(vehicle_yaw) || !PX4_ISFINITE(mount_yaw) ||
		!PX4_ISFINITE(gimbal_pitch) || !PX4_ISFINITE(gimbal_yaw)) {
		return false;
	}

	matrix::Quatf q_heading(matrix::Eulerf(0, 0, vehicle_yaw));
	matrix::Quatf q_mount_yaw(matrix::Eulerf(0, 0, mount_yaw));
	matrix::Quatf q_pitch(matrix::Eulerf(0, gimbal_pitch, 0));
	matrix::Quatf q_yaw(matrix::Eulerf(0, 0, gimbal_yaw));
	matrix::Quatf q_gimbal_g0 = q_yaw * q_pitch;

	// G0 is heading-level: it follows vehicle yaw and mount yaw, but keeps roll/pitch level to NED.
	pose.q_gimbal_ned = q_heading * q_mount_yaw * q_gimbal_g0;

	matrix::Eulerf euler_gimbal_ned(pose.q_gimbal_ned);
	pose.gimbal_roll_ned = euler_gimbal_ned.phi();
	pose.gimbal_pitch_ned = euler_gimbal_ned.theta();
	pose.gimbal_yaw_ned = euler_gimbal_ned.psi();
	pose.valid = true;
	return true;
}

bool build_guidance_command(const VehicleGuidanceState &veh, const matrix::Vector3f &target_vec_ned,
			float max_forward_v, float max_vz, const DescentShapingConfig &shaping, GuidanceCommand &cmd)
{
	if (!veh.valid || !PX4_ISFINITE(max_forward_v) || !PX4_ISFINITE(max_vz) ||
		max_forward_v < 0.0f || max_vz < 0.0f || target_vec_ned.norm() <= 1e-3f) {
		return false;
	}

	if (!PX4_ISFINITE(shaping.local_height_stop) || !PX4_ISFINITE(shaping.local_height_full) ||
		!PX4_ISFINITE(shaping.pitch_stop_rad) || !PX4_ISFINITE(shaping.pitch_full_rad) ||
		shaping.local_height_stop < 0.0f || shaping.local_height_full <= shaping.local_height_stop ||
		shaping.pitch_stop_rad < 0.0f || shaping.pitch_full_rad <= shaping.pitch_stop_rad) {
		return false;
	}

	matrix::Vector3f target_dir_ned = target_vec_ned;
	target_dir_ned.normalize();

	cmd.vx_ned = target_dir_ned(0) * max_forward_v;
	cmd.vy_ned = target_dir_ned(1) * max_forward_v;
	const float vz_unconstrained_ned = target_dir_ned(2) * max_forward_v;
	cmd.vz_raw_ned = math::constrain(vz_unconstrained_ned, -max_vz, max_vz);
	cmd.vz_saturated = fabsf(vz_unconstrained_ned - cmd.vz_raw_ned) > 1e-4f;
	cmd.vz_ned = cmd.vz_raw_ned;
	cmd.local_height = veh.local_height;
	cmd.local_height_valid = veh.local_height_valid;
	const float horizontal_norm = sqrtf(target_dir_ned(0) * target_dir_ned(0) + target_dir_ned(1) * target_dir_ned(1));
	cmd.los_pitch_down_rad = atan2f(target_dir_ned(2), horizontal_norm);

	if (target_dir_ned(2) > 1e-4f) {
		cmd.target_relation = TargetVerticalRelation::BELOW;

		if (veh.local_height_valid) {
			cmd.height_scale = smoothstep(shaping.local_height_stop, shaping.local_height_full, veh.local_height);
			cmd.angle_scale = smoothstep(shaping.pitch_stop_rad, shaping.pitch_full_rad, cmd.los_pitch_down_rad);

			if (shaping.enabled) {
				cmd.descent_shaping_active = true;
				cmd.descent_scale = math::max(cmd.height_scale, cmd.angle_scale);
				cmd.vz_ned = cmd.vz_raw_ned * cmd.descent_scale;
			}
		}

	} else if (target_dir_ned(2) < -1e-4f) {
		cmd.target_relation = TargetVerticalRelation::ABOVE;
	}

	cmd.target_roll = veh.veh_roll;
	cmd.target_pitch = veh.veh_pitch;
	cmd.target_yaw = matrix::wrap_pi(atan2f(target_dir_ned(1), target_dir_ned(0)));
	cmd.target_yaw_rate = 0.0f;
	cmd.valid = true;
	return true;
}

} // namespace attack_vision_guidance
