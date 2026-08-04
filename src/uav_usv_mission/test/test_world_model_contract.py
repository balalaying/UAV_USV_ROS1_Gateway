import math

from uav_usv_mission.world_model_contract import normalize_sensor_sources
from uav_usv_mission.world_model_contract import planar_course
from uav_usv_mission.world_model_contract import quaternion_yaw
from uav_usv_mission.world_model_contract import stream_is_usable


def test_quaternion_yaw_and_course():
    yaw = quaternion_yaw(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    assert math.isclose(yaw, math.pi / 2, abs_tol=1.0e-6)
    assert math.isclose(planar_course(0.0, 2.0), math.pi / 2)
    assert math.isclose(planar_course(0.0, 0.0, fallback=yaw), yaw)


def test_only_fresh_map_stream_is_usable():
    base = {
        'online': True,
        'stale': False,
        'header': {'frame_id': 'map'},
        'objects': [],
    }
    assert stream_is_usable(base, 'map')
    assert not stream_is_usable({**base, 'stale': True}, 'map')
    assert not stream_is_usable(
        {**base, 'header': {'frame_id': 'usv_01/mid360_link'}}, 'map'
    )
    assert not stream_is_usable({**base, 'online': False}, 'map')


def test_sensor_source_labels_are_stable_and_unique():
    assert normalize_sensor_sources(
        'usv_01_mid360+uav_01_camera',
        ['USV_LIDAR', 'CAMERA', 'FUSION'],
    ) == [
        'USV_01_MID360',
        'UAV_01_CAMERA',
        'USV_LIDAR',
        'CAMERA',
    ]
    assert normalize_sensor_sources('', []) == ['UNKNOWN']
