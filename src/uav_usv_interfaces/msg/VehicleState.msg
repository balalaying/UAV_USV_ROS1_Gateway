uint8 TYPE_UNKNOWN=0
uint8 TYPE_UAV=1
uint8 TYPE_USV=2

std_msgs/Header header
string vehicle_id
uint8 vehicle_type
bool online
bool armed
string mode
geometry_msgs/Pose pose
geometry_msgs/Twist twist
float32 battery_percent
string active_command_id
string status_text
