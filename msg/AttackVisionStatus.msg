uint8 MODULE_STATE_HOLD = 0
uint8 MODULE_STATE_SWITCHING_TO_OFFBOARD = 1
uint8 MODULE_STATE_OFFBOARD = 2

uint8 TARGET_RELATION_LEVEL = 0
uint8 TARGET_RELATION_BELOW = 1
uint8 TARGET_RELATION_ABOVE = 2

bool lock_active
bool rc_offboard
bool external_mission_active
bool allow_takeover
bool frame_valid
int16 pix_offset_x
int16 pix_offset_y
int16 gimbal_roll_raw_deg100
int16 gimbal_pitch_raw_deg100
int16 gimbal_yaw_raw_deg100
float32 gimbal_roll_rad
float32 gimbal_pitch_rad
float32 gimbal_yaw_rad
bool los_gimbal_valid
float32 los_gimbal_x
float32 los_gimbal_y
float32 los_gimbal_z
bool target_vec_ned_valid
float32 target_vec_ned_x
float32 target_vec_ned_y
float32 target_vec_ned_z
bool guidance_command_valid
bool local_height_valid
uint8 target_relation
float32 local_height
float32 los_pitch_down_rad
float32 vz_raw_ned
float32 height_scale
float32 angle_scale
float32 descent_scale
float32 vz_cmd_ned
uint64 guidance_timestamp
uint32 frame_age_ms
uint8 module_state
uint64 timestamp
