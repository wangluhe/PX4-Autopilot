#include <gtest/gtest.h>
#include <lib/mathlib/math/Limits.hpp>

#include "attack_vision_guidance.hpp"

using attack_vision_guidance::build_guidance_command;

static DescentShapingConfig shaping_config(bool enabled = true)
{
	DescentShapingConfig config{};
	config.enabled = enabled;
	config.local_height_stop = 0.8f;
	config.local_height_full = 2.0f;
	config.pitch_stop_rad = math::radians(3.0f);
	config.pitch_full_rad = math::radians(12.0f);
	return config;
}

static matrix::Vector3f los_at_pitch(float pitch_deg)
{
	const float pitch = math::radians(pitch_deg);
	return matrix::Vector3f(cosf(pitch), 0.0f, sinf(pitch));
}

TEST(AttackVisionGuidance, ShapesOnlyValidLowTargetDescent)
{
	VehicleGuidanceState vehicle{};
	vehicle.valid = true;
	vehicle.local_height = 0.5f;
	vehicle.local_height_valid = true;

	GuidanceCommand below{};
	ASSERT_TRUE(build_guidance_command(vehicle, los_at_pitch(5.0f), 1.5f, 1.5f, shaping_config(), below));
	EXPECT_EQ(below.target_relation, TargetVerticalRelation::BELOW);
	EXPECT_GT(below.vz_raw_ned, 0.0f);
	EXPECT_GT(below.vz_ned, 0.0f);
	EXPECT_LT(below.vz_ned, below.vz_raw_ned);

	GuidanceCommand above{};
	ASSERT_TRUE(build_guidance_command(vehicle, los_at_pitch(-5.0f), 1.5f, 1.5f, shaping_config(), above));
	EXPECT_EQ(above.target_relation, TargetVerticalRelation::ABOVE);
	EXPECT_FLOAT_EQ(above.vz_ned, above.vz_raw_ned);

	vehicle.local_height_valid = false;
	GuidanceCommand no_height{};
	ASSERT_TRUE(build_guidance_command(vehicle, los_at_pitch(5.0f), 1.5f, 1.5f, shaping_config(), no_height));
	EXPECT_FLOAT_EQ(no_height.vz_ned, no_height.vz_raw_ned);
}
