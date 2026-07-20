# Source bitmask. Multiple bits indicate a fused track.
uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_LIDAR=1
uint8 SOURCE_CAMERA=2
uint8 SOURCE_AIS=4
uint8 SOURCE_FUSED=8

# Coarse semantic classification shared by all perception sources.
uint8 CLASS_UNKNOWN=0
uint8 CLASS_VESSEL=1
uint8 CLASS_BUOY=2
uint8 CLASS_DEBRIS=3
uint8 CLASS_LANDMARK=4

unique_identifier_msgs/UUID uuid
string track_id
builtin_interfaces/Time first_seen
builtin_interfaces/Time last_update
uint8 source_mask
uint8 classification
# Human-readable semantic metadata. These fields extend the enum/bitmask
# contract without changing how existing consumers interpret it.
string class_name
float32 class_confidence
string sensor_source
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
geometry_msgs/Vector3 dimensions
float32 confidence
uint32 mmsi

# Affiliation is independent from semantic classification. Legacy publishers
# leave these fields at UNKNOWN / 0.0 without changing their behaviour.
uint8 AFFILIATION_UNKNOWN=0
uint8 AFFILIATION_FRIENDLY=1
uint8 AFFILIATION_HOSTILE=2
uint8 AFFILIATION_NEUTRAL=3
uint8 affiliation
float32 affiliation_confidence

# Geometry diagnostics for camera-guided LiDAR observations.
uint32 bbox_point_count
float32 association_score
