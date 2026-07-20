import cv2
import numpy as np

from camera_lidar_association_node import association_score
from camera_lidar_association_node import marker_box
from camera_lidar_association_node import project_box
from camera_lidar_association_node import rectangle_iou
from geometry_msgs.msg import Point
from geometry_msgs.msg import Transform
from sensor_msgs.msg import CameraInfo
from usv_camera_detection_node import detect_vessel_candidates
from visualization_msgs.msg import Marker


def test_camera_detector_finds_navigation_light_candidate():
    image = np.zeros((180, 320, 3), dtype=np.uint8)
    image[:] = (120, 80, 30)
    cv2.circle(image, (210, 82), 4, (30, 255, 60), -1)
    candidates = detect_vessel_candidates(image, min_pixels=2, padding=4)
    assert len(candidates) == 1
    left, top, right, bottom, confidence = candidates[0]
    assert left < 210 < right
    assert top < 82 < bottom
    assert confidence > 0.5


def test_camera_detector_finds_blue_vessel_without_selecting_water():
    image = np.full((180, 320, 3), (185, 179, 170), dtype=np.uint8)
    cv2.rectangle(image, (205, 82), (224, 88), (86, 63, 32), -1)
    candidates = detect_vessel_candidates(image, min_pixels=2, padding=4)
    assert len(candidates) == 1
    left, top, right, bottom, _confidence = candidates[0]
    assert left < 214 < right
    assert top < 85 < bottom


def _box_marker():
    marker = Marker()
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.id = 3
    marker.pose.position.x = 20.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0
    corners = (
        (-3.0, -1.0, -1.0), (-3.0, 1.0, -1.0),
        (3.0, 1.0, -1.0), (3.0, -1.0, -1.0),
        (-3.0, -1.0, 1.0), (-3.0, 1.0, 1.0),
        (3.0, 1.0, 1.0), (3.0, -1.0, 1.0),
    )
    for start, end in (
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ):
        for index in (start, end):
            point = Point()
            point.x, point.y, point.z = corners[index]
            marker.points.append(point)
    return marker


def test_lidar_box_projects_into_camera_image():
    box = marker_box(_box_marker())
    assert box['dimensions'].tolist() == [6.0, 2.0, 2.0]
    transform = Transform()
    transform.rotation.w = 1.0
    info = CameraInfo()
    info.width = 320
    info.height = 180
    info.k = [277.0, 0.0, 159.5, 0.0, 277.0, 89.5, 0.0, 0.0, 1.0]
    projection = project_box(box, transform, info)
    assert projection is not None
    assert projection[0] < 159.5 < projection[2]
    assert projection[1] < 89.5 < projection[3]


def test_association_accepts_overlapping_rectangles():
    class Center:
        class Position:
            x = 160.0
            y = 90.0
        position = Position()

    class Box:
        center = Center()
        size_x = 24.0
        size_y = 18.0

    class Detection:
        bbox = Box()

    projection = (145.0, 76.0, 175.0, 104.0)
    assert rectangle_iou((148.0, 81.0, 172.0, 99.0), projection) > 0.4
    assert association_score(Detection(), projection, 28.0) > 0.08
