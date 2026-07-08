uint8 COMMAND_HOLD=0
uint8 COMMAND_NAVIGATE=1
uint8 COMMAND_TAKEOFF=2
uint8 COMMAND_PATROL=3
uint8 COMMAND_RETURN=4
uint8 COMMAND_LAND=5
uint8 COMMAND_EMERGENCY_STOP=255

std_msgs/Header header
string command_id
string vehicle_id
string lease_id
uint8 command_type
uint8 priority
builtin_interfaces/Time expires_at
geometry_msgs/Pose target_pose
float32[] parameters
