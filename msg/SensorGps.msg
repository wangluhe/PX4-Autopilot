# GPS position in WGS84 coordinates.
# the field 'timestamp' is for the position & velocity (microseconds)
uint64 timestamp		# time since system start (microseconds) # 自系统启动以来的时间戳（微秒）
uint64 timestamp_sample

uint32 device_id                # unique device ID for the sensor that does not change between power cycles # 传感器唯一ID，断电重启后通常保持不变

float64 latitude_deg		# Latitude in degrees, allows centimeter level RTK precision # 纬度，单位度
float64 longitude_deg		# Longitude in degrees, allows centimeter level RTK precision # 经度，单位度
float64 altitude_msl_m		# Altitude above MSL, meters # 海拔高程，基于平均海平面，单位米
float64 altitude_ellipsoid_m	# Altitude above Ellipsoid, meters # 椭球高，单位米

float32 s_variance_m_s		# GPS speed accuracy estimate, (metres/sec) # 速度精度估计，单位 m/s
float32 c_variance_rad		# GPS course accuracy estimate, (radians) # 航向精度估计，单位 rad
uint8 FIX_TYPE_NONE                   = 1       # Value 0 is also valid to represent no fix.
uint8 FIX_TYPE_2D                     = 2
uint8 FIX_TYPE_3D                     = 3
uint8 FIX_TYPE_RTCM_CODE_DIFFERENTIAL = 4
uint8 FIX_TYPE_RTK_FLOAT              = 5
uint8 FIX_TYPE_RTK_FIXED              = 6
uint8 FIX_TYPE_EXTRAPOLATED           = 8
uint8 fix_type                  # Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.

float32 eph			# GPS horizontal position accuracy (metres) # 水平位置精度，单位米
float32 epv			# GPS vertical position accuracy (metres) # 垂直位置精度，单位米

float32 hdop			# Horizontal dilution of precision # 水平精度因子
float32 vdop			# Vertical dilution of precision # 垂直精度因子

int32 noise_per_ms		# GPS noise per millisecond # 每毫秒噪声计数
uint16 automatic_gain_control   # Automatic gain control monitor # 自动增益控制监测值

uint8 JAMMING_STATE_UNKNOWN  = 0
uint8 JAMMING_STATE_OK       = 1
uint8 JAMMING_STATE_WARNING  = 2
uint8 JAMMING_STATE_CRITICAL = 3
uint8 jamming_state		# indicates whether jamming has been detected or suspected by the receivers. O: Unknown, 1: OK, 2: Warning, 3: Critical # 干扰状态
int32 jamming_indicator		# indicates jamming is occurring # 干扰强度指示

uint8 SPOOFING_STATE_UNKNOWN   = 0
uint8 SPOOFING_STATE_NONE      = 1
uint8 SPOOFING_STATE_INDICATED = 2
uint8 SPOOFING_STATE_MULTIPLE  = 3
uint8 spoofing_state		# indicates whether spoofing has been detected or suspected by the receivers. O: Unknown, 1: OK, 2: Warning, 3: Critical # 欺骗检测状态

float32 vel_m_s			# GPS ground speed, (metres/sec) # 地速，单位 m/s
float32 vel_n_m_s		# GPS North velocity, (metres/sec) # 北向速度，单位 m/s
float32 vel_e_m_s		# GPS East velocity, (metres/sec) # 东向速度，单位 m/s
float32 vel_d_m_s		# GPS Down velocity, (metres/sec) # 下向速度，单位 m/s
float32 cog_rad			# Course over ground (NOT heading, but direction of movement), -PI..PI, (radians) # 运动航迹角，不是机头航向
bool vel_ned_valid		# True if NED velocity is valid # NED 速度是否有效

int32 timestamp_time_relative	# timestamp + timestamp_time_relative = Time of the UTC timestamp since system start, (microseconds) # UTC 时间相对系统启动时间的偏移
uint64 time_utc_usec		# Timestamp (microseconds, UTC), this is the timestamp which comes from the gps module. It might be unavailable right after cold start, indicated by a value of 0 # GPS 提供的 UTC 时间戳

uint8 satellites_used		# Number of satellites used # 当前参与定位的卫星数

float32 heading			# heading angle of XYZ body frame rel to NED. Set to NaN if not available and updated (used for dual antenna GPS), (rad, [-PI, PI]) # 双天线 GPS 航向角
float32 heading_offset		# heading offset of dual antenna array in body frame. Set to NaN if not applicable. (rad, [-PI, PI]) # 双天线安装偏角
float32 heading_accuracy	# heading accuracy (rad, [0, 2PI]) # 航向角精度

float32 rtcm_injection_rate	# RTCM message injection rate Hz # RTCM 注入频率
uint8 selected_rtcm_instance	# uorb instance that is being used for RTCM corrections # 当前使用的 RTCM 实例

bool rtcm_crc_failed		# RTCM message CRC failure detected # RTCM 校验是否失败

uint8 RTCM_MSG_USED_UNKNOWN = 0
uint8 RTCM_MSG_USED_NOT_USED = 1
uint8 RTCM_MSG_USED_USED = 2
uint8 rtcm_msg_used		# Indicates if the RTCM message was used successfully by the receiver # RTCM 是否被接收机成功使用

# TOPICS sensor_gps vehicle_gps_position
