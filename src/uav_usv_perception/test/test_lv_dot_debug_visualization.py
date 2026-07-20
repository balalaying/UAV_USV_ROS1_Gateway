from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from lv_dot_debug_visualization_node import cluster_markers_from_bboxes


def test_bbox_debug_marker_becomes_cluster_center_marker():
    clear = Marker()
    clear.action = Marker.DELETEALL
    bbox = Marker()
    bbox.header.frame_id = 'map'
    bbox.ns = 'lv_dot_ros2/lidar_bboxes'
    bbox.id = 7
    bbox.action = Marker.ADD
    bbox.type = Marker.LINE_LIST
    bbox.pose.position.x = 3.0
    bbox.pose.position.y = -4.0
    bbox.pose.position.z = 1.2
    bbox.pose.orientation.w = 1.0
    bbox.text = 'cluster_id=7;points=42;cluster_ms=1.0'

    result = cluster_markers_from_bboxes(MarkerArray(markers=[clear, bbox]))

    assert len(result.markers) == 2
    assert result.markers[0].action == Marker.DELETEALL
    cluster = result.markers[1]
    assert cluster.type == Marker.SPHERE
    assert cluster.id == 7
    assert cluster.header.frame_id == 'map'
    assert cluster.pose.position.x == 3.0
    assert cluster.pose.position.y == -4.0
    assert cluster.text == 'cluster_id=7;points=42'
