import math

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import pytest
from uav_usv_interfaces.msg import TrackedObject
from uav_usv_interfaces.msg import TrackedObjectArray
from uav_usv_mission.perception_topdown import marker_segments_3d
from uav_usv_mission.perception_topdown import marker_segments_xy
from uav_usv_mission.perception_topdown import TopDownVisualizationModel
from visualization_msgs.msg import Marker


def test_cube_marker_projects_to_rotated_topdown_rectangle():
    marker = Marker()
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.scale.x = 4.0
    marker.scale.y = 2.0
    marker.pose.orientation.z = math.sin(math.pi / 4.0)
    marker.pose.orientation.w = math.cos(math.pi / 4.0)

    segments = marker_segments_xy(marker)
    points = [point for segment in segments for point in segment]

    assert len(segments) == 4
    assert max(point[0] for point in points) == pytest.approx(1.0)
    assert min(point[0] for point in points) == pytest.approx(-1.0)
    assert max(point[1] for point in points) == pytest.approx(2.0)
    assert min(point[1] for point in points) == pytest.approx(-2.0)


def test_cube_marker_preserves_twelve_edges_and_height_in_3d():
    marker = Marker()
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.scale.x = 4.0
    marker.scale.y = 2.0
    marker.scale.z = 3.0
    marker.pose.position.z = 2.0
    marker.pose.orientation.w = 1.0

    segments = marker_segments_3d(marker)
    points = [point for segment in segments for point in segment]

    assert len(segments) == 12
    assert min(point[2] for point in points) == pytest.approx(0.5)
    assert max(point[2] for point in points) == pytest.approx(3.5)


def test_model_keeps_latest_points_and_track_history():
    model = TopDownVisualizationModel()
    points = PoseArray()
    pose = Pose()
    pose.position.x = 3.0
    pose.position.y = -2.0
    points.poses.append(pose)
    model.update_points(points)

    first = TrackedObjectArray()
    first.header.frame_id = 'map'
    tracked = TrackedObject()
    tracked.track_id = 'target'
    tracked.pose.pose.position.x = 1.0
    tracked.confidence = 0.8
    first.objects.append(tracked)
    model.update_tracks('dynamic', first)

    second = TrackedObjectArray()
    second.header.frame_id = 'map'
    tracked = TrackedObject()
    tracked.track_id = 'target'
    tracked.pose.pose.position.x = 2.0
    tracked.confidence = 0.9
    second.objects.append(tracked)
    model.update_tracks('dynamic', second)

    snapshot = model.snapshot()

    assert snapshot['points'].tolist() == [[3.0, -2.0]]
    assert snapshot['point_heights'].tolist() == [0.0]
    assert snapshot['tracks']['dynamic']['target']['x'] == 2.0
    assert snapshot['histories'][('dynamic', 'target')] == [
        (1.0, 0.0),
        (2.0, 0.0),
    ]
