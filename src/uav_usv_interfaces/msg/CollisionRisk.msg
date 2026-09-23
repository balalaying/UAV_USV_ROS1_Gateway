uint8 RISK_NONE=0
uint8 RISK_MONITOR=1
uint8 RISK_CAUTION=2
uint8 RISK_DANGER=3
uint8 RISK_EMERGENCY=4

std_msgs/Header header
uuid_msgs/UniqueID target_uuid
string track_id
float32 current_distance_m
float32 dcpa_m
float32 tcpa_s
float32 relative_bearing_rad
uint8 risk_level
bool valid
