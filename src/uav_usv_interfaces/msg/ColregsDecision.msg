uint8 ENCOUNTER_NONE=0
uint8 ENCOUNTER_HEAD_ON=1
uint8 ENCOUNTER_CROSSING_STARBOARD=2
uint8 ENCOUNTER_CROSSING_PORT=3
uint8 ENCOUNTER_OVERTAKING=4

uint8 ACTION_MAINTAIN=0
uint8 ACTION_TURN_STARBOARD=1
uint8 ACTION_TURN_PORT=2
uint8 ACTION_SLOW_DOWN=3
uint8 ACTION_STOP=4
uint8 ACTION_REPLAN=5

std_msgs/Header header
uuid_msgs/UniqueID target_uuid
string track_id
uint8 encounter_type
uint8 recommended_action
bool ownship_give_way
float32 recommended_speed_mps
float32 heading_change_rad
float32 valid_for_s
string reason
