int32 POSITION_TYPE_NONE = 0
int32 POSITION_TYPE_FIXEDPOS = 1
int32 POSITION_TYPE_FIXEDHEIGHT = 2
int32 POSITION_TYPE_DOPPLER_VELOCITY = 8
int32 POSITION_TYPE_SINGLE = 16
int32 POSITION_TYPE_PSRDIFF = 17
int32 POSITION_TYPE_SBAS = 18
#int32 POSITION_TYPE_WAAS = 18
int32 POSITION_TYPE_PROPAGATED = 19
int32 POSITION_TYPE_OMNISTAR = 20
int32 POSITION_TYPE_L1_FLOAT = 32
int32 POSITION_TYPE_IONOFREE_FLOAT = 33
int32 POSITION_TYPE_NARROW_FLOAT = 34
int32 POSITION_TYPE_L1_INT = 48
int32 POSITION_TYPE_WIDE_INT = 49
int32 POSITION_TYPE_NARROW_INT = 50
int32 POSITION_TYPE_OMNISTAR_HP = 64
int32 POSITION_TYPE_OMNISTAR_XP = 65
#int32 POSITION_TYPE_PPP_CONVERGING = 68
#int32 POSITION_TYPE_PPP = 69
#int32 POSITION_TYPE_OPERATIONAL = 70
#int32 POSITION_TYPE_WARNING = 71
#int32 POSITION_TYPE_OUT_OF_BOUNDS = 72


std_msgs/Header header

# position
float64 latitude
float64 longitude
float64 altitude

float64 utm_east
float64 utm_north

float32 latitude_std
float32 longitude_std
float32 altitude_std

# velocity
float64 north_velocity
float64 east_velocity
float64 up_velocity

# attitude
float64 roll
float64 pitch
float64 yaw

float64 x
float64 y
float64 z
float64 w

# angular velocity
float64 roll_rate
float64 pitch_rate
float64 yaw_rate

# acceleration
float64 ax
float64 ay
float64 az

# INS570D status
# initialization status
int32 ins_gpsflag_pos
int32 ins_gpsflag_heading
int32 ins_status

# gps_satellitenum
int32 ins_numsv

# RTK_delay
float64 ins_gps_age

# car_status
uint8 front_wheelspeed_status
uint8 rear_wheelspeed_status
uint8 gear_status
uint8 street_corner