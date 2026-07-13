uint8 ROLE_UNASSIGNED=0
uint8 ROLE_AIR_OBSERVER=1
uint8 ROLE_SURFACE_INTERCEPTOR=2

string vehicle_id
uint8 vehicle_type
uint8 role_type
string role_name
geometry_msgs/Pose target_pose
float32 assignment_cost
bool active
string status
