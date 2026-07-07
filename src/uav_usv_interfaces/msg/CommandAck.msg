uint8 STATUS_RECEIVED=0
uint8 STATUS_ACCEPTED=1
uint8 STATUS_EXECUTING=2
uint8 STATUS_SUCCEEDED=3
uint8 STATUS_REJECTED=4
uint8 STATUS_FAILED=5
uint8 STATUS_CANCELED=6

std_msgs/Header header
string command_id
string vehicle_id
uint8 status
float32 progress
string message
