import numpy as np
from geometry_msgs.msg import Transform

from uav_usv_mission.lv_dot_debug_visualization import LvDotDebugModel


def test_debug_model_keeps_raw_and_filtered_clouds_separate():
    model = LvDotDebugModel()
    model.set_cloud_layer_enabled('raw', True)
    raw = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    filtered = np.array([[4.0, 5.0, 6.0]])

    model.update_cloud('raw', raw)
    model.update_cloud('filtered', filtered)
    snapshot = model.snapshot()

    assert np.allclose(snapshot['clouds']['raw'], raw)
    assert np.allclose(snapshot['clouds']['filtered'], filtered)
    assert snapshot['generation'] == 2


def test_debug_model_keeps_camera_lidar_calibration_roi_separate():
    model = LvDotDebugModel()
    model.set_cloud_layer_enabled('calibration_roi', True)
    roi = np.array([[8.0, 1.0, 0.8], [8.1, 1.1, 1.0]])

    model.update_cloud('calibration_roi', roi)
    snapshot = model.snapshot()

    assert np.allclose(snapshot['clouds']['calibration_roi'], roi)
    assert snapshot['clouds']['raw'].shape == (0, 3)


def test_debug_model_keeps_base_and_mid360_transforms():
    model = LvDotDebugModel()
    transform = Transform()
    transform.translation.x = 12.0
    transform.translation.y = -3.5
    transform.translation.z = 2.2
    transform.rotation.w = 1.0

    model.update_frame('radar', 'usv_01/mid360_link', transform)
    frame = model.snapshot()['frames']['radar']

    assert frame['frame_id'] == 'usv_01/mid360_link'
    assert frame['x'] == 12.0
    assert frame['y'] == -3.5
    assert frame['z'] == 2.2
    assert frame['qw'] == 1.0
