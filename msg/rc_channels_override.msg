uint8 RC_CHANNELS_MODE_IGNORE=0
uint8 RC_CHANNELS_MODE_OVERRIDE_MAV=1
uint8 RC_CHANNELS_MODE_OVERRIDE_RC=2
uint8 RC_CHANNELS_MODE_OVERRIDE_ADD_2_RC=3
uint64 timestamp						# Timestamp in microseconds since boot time
uint64 timestamp_last_valid					# Timestamp of last valid RC signal
float32 throttle                                                # throttle override
float32 pitch                                                   # pitch override
float32 roll                                                    # roll override
float32 yaw                                                     # yaw override

uint8 throttle_mode
uint8 pitch_mode
uint8 roll_mode
uint8 yaw_mode
