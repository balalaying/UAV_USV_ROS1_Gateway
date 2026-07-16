import random

from builtin_interfaces.msg import Time
from sensor_msgs.msg import CameraInfo
from uav_usv_interfaces.msg import TrackedObject

from uav_visual_observation_node import make_camera_observation
from uav_visual_observation_node import project_camera_point


def _camera_info():
    info = CameraInfo()
    info.width = 320
    info.height = 180
    info.k = [135.0, 0.0, 160.0, 0.0, 135.0, 90.0, 0.0, 0.0, 1.0]
    return info


def test_gazebo_x_axis_projection_uses_camera_intrinsics():
    projected = project_camera_point((10.0, 1.0, -2.0), _camera_info(), 'x')

    assert projected == (146.5, 117.0, 10.0)
    assert project_camera_point((-1.0, 0.0, 0.0), _camera_info(), 'x') is None


def test_camera_observation_uses_sensor_independent_contract():
    truth = TrackedObject()
    truth.track_id = 'target_vessel'
    truth.classification = TrackedObject.CLASS_VESSEL
    truth.pose.pose.position.x = 12.0
    truth.twist.twist.linear.x = 1.5
    stamp = Time(sec=100, nanosec=20)

    observation = make_camera_observation(
        truth=truth,
        stamp=stamp,
        vehicle_id='uav_01',
        position_noise=0.0,
        velocity_noise=0.0,
        position_variance=0.25,
        velocity_variance=0.04,
        confidence=0.82,
        random_source=random.Random(1),
        first_seen=stamp,
    )

    assert observation.track_id == 'uav_01_camera_target_vessel'
    assert observation.source_mask == TrackedObject.SOURCE_CAMERA
    assert observation.classification == TrackedObject.CLASS_VESSEL
    assert observation.pose.pose.position.x == 12.0
    assert observation.twist.twist.linear.x == 1.5
    assert observation.pose.covariance[0] == 0.25
    assert observation.twist.covariance[0] == 0.04
    assert observation.confidence == 0.82
    assert observation.last_update == stamp
    assert any(observation.uuid.uuid)
