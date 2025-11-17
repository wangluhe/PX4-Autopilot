uint64 timestamp        # time since system start (microseconds)
uint8 guidance_state    # 0: waiting, 1: active, 2: lost target, 3: error
bool target_locked      # target is locked by vision pod
bool guidance_active    # guidance control is active
int16 off_target_azimuth    # azimuth offset in pixels
int16 off_target_elevation  # elevation offset in pixels
float32 velocity_x          # current velocity command X
float32 velocity_y          # current velocity command Y
