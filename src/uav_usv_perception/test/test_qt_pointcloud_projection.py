from types import SimpleNamespace

import numpy as np

from qt_pointcloud_projection_node import project_points
from qt_pointcloud_projection_node import quaternion_rotation_matrix
from qt_pointcloud_projection_node import xyz_array_to_pointcloud
from std_msgs.msg import Header


def test_projection_transforms_filters_and_limits_points():
    points = np.array([
        [0.0, 0.0, 0.0],
        [0.05, 0.05, 0.0],
        [1.0, 0.0, 1.0],
        [2.0, 0.0, 4.0],
        [np.nan, 1.0, 0.0],
    ])

    result = project_points(
        points,
        np.eye(3),
        np.array([10.0, -2.0, 0.0]),
        min_z=-0.5,
        max_z=2.0,
        voxel_size=0.2,
        max_points=2,
    )

    assert result.shape == (2, 3)
    assert np.allclose(result[0], [10.0, -2.0, 0.0])
    assert np.allclose(result[1], [11.0, -2.0, 1.0])


def test_quaternion_rotation_matrix_rotates_xy_ninety_degrees():
    quaternion = SimpleNamespace(
        x=0.0,
        y=0.0,
        z=np.sin(np.pi / 4.0),
        w=np.cos(np.pi / 4.0),
    )

    rotation = quaternion_rotation_matrix(quaternion)
    result = rotation @ np.array([1.0, 0.0, 0.0])

    assert np.allclose(result, [0.0, 1.0, 0.0], atol=1e-8)


def test_xyz_array_is_packed_as_compact_pointcloud2():
    points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    message = xyz_array_to_pointcloud(Header(frame_id='map'), points)

    assert message.header.frame_id == 'map'
    assert message.width == 2
    assert message.point_step == 12
    assert message.row_step == 24
    assert np.allclose(
        np.frombuffer(message.data, dtype='<f4').reshape((-1, 3)), points
    )
