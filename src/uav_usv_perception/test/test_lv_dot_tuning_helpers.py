import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from adapters.lv_dot_adapter import LvDotAdapter
from lv_dot_target_motion import motion_command
from mid360_preprocessor import Mid360Preprocessor


def _command(profile, elapsed):
    return motion_command(
        profile=profile,
        elapsed=elapsed,
        constant_speed=0.45,
        constant_leg_seconds=12.0,
        turn_speed=0.8,
        turn_radius=2.0,
        acceleration_initial=0.15,
        acceleration=0.08,
        acceleration_max=0.85,
    )


def test_constant_profile_reverses_inside_tuning_window():
    assert _command('constant', 0.0) == (0.45, 0.0)
    assert _command('constant', 12.1) == (-0.45, 0.0)
    assert _command('constant', 24.1) == (0.45, 0.0)


def test_turn_and_acceleration_profiles_preserve_radius():
    speed, yaw_rate = _command('turn', 5.0)
    assert np.isclose(speed / yaw_rate, 2.0)
    early_speed, early_yaw_rate = _command('acceleration', 0.0)
    late_speed, late_yaw_rate = _command('acceleration', 20.0)
    assert early_speed < late_speed <= 0.85
    assert np.isclose(early_speed / early_yaw_rate, 2.0)
    assert np.isclose(late_speed / late_yaw_rate, 2.0)


def test_mid360_height_filter_rejects_water_and_keeps_target():
    preprocessor = Mid360Preprocessor.__new__(Mid360Preprocessor)
    preprocessor.min_range = 0.5
    preprocessor.max_range = 20.0
    preprocessor.min_z = -1.75
    preprocessor.max_z = 4.0
    preprocessor.crop_self = False
    preprocessor.voxel_size = 0.0
    points = np.array([
        [4.0, 0.0, -2.05],
        [5.0, 0.0, -1.20],
        [5.0, 0.2, -0.40],
        [25.0, 0.0, 0.0],
    ], dtype=np.float32)
    indices = preprocessor._filter_indices(points)
    assert indices.tolist() == [1, 2]


def test_lv_dot_adapter_deduplicates_nearby_boxes_by_volume():
    adapter = LvDotAdapter.__new__(LvDotAdapter)
    adapter.deduplication_distance = 1.0

    small = Marker()
    small.pose.position.x = 4.0
    small.points = [Point(x=-0.5, y=-0.5, z=-0.5),
                    Point(x=0.5, y=0.5, z=0.5)]
    large = Marker()
    large.pose.position.x = 4.2
    large.points = [Point(x=-1.0, y=-1.0, z=-1.0),
                    Point(x=1.0, y=1.0, z=1.0)]
    separate = Marker()
    separate.pose.position.x = 8.0
    separate.points = [Point(x=-0.5, y=-0.5, z=-0.5),
                       Point(x=0.5, y=0.5, z=0.5)]

    selected = adapter._deduplicated_markers([small, large, separate])

    assert selected == [large, separate]
