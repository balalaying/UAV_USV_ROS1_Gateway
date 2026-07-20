uint8 AFFILIATION_UNKNOWN=0
uint8 AFFILIATION_FRIENDLY=1
uint8 AFFILIATION_HOSTILE=2
uint8 AFFILIATION_NEUTRAL=3

std_msgs/Header header
string detection_id
float32 center_x
float32 center_y
float32 size_x
float32 size_y
string class_name
float32 class_confidence
uint8 affiliation
float32 affiliation_confidence
