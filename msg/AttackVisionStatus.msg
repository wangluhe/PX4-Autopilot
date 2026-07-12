uint8 MODULE_STATE_HOLD = 0
uint8 MODULE_STATE_SWITCHING_TO_OFFBOARD = 1
uint8 MODULE_STATE_OFFBOARD = 2

bool lock_active
bool rc_offboard
bool external_mission_active
bool allow_takeover
bool frame_valid
int16 pix_offset_x
int16 pix_offset_y
uint32 frame_age_ms
uint8 module_state
uint64 timestamp
