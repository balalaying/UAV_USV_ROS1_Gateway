import math

import cv2
import numpy as np

from vision_guided_core import AFFILIATION_FRIENDLY
from vision_guided_core import AFFILIATION_HOSTILE
from vision_guided_core import AFFILIATION_NEUTRAL
from vision_guided_core import AFFILIATION_UNKNOWN
from vision_guided_core import AffiliationTemporalFilter
from vision_guided_core import BboxSmoother
from vision_guided_core import bbox_is_plausible
from vision_guided_core import dbscan
from vision_guided_core import depth_filter
from vision_guided_core import complete_occluded_bbox
from vision_guided_core import project_camera_points
from vision_guided_core import robust_oriented_bbox
from vision_guided_core import roi_indices
from vision_guided_core import select_cluster
from vision_guided_core import unproject_camera_pixels
from usv_camera_detection_node import detect_affiliated_candidates


def _identity_test_image():
    image = np.zeros((240, 400, 3), dtype=np.uint8)
    image[:] = (100, 100, 100)
    cv2.rectangle(image, (45, 80), (70, 92), (255, 0, 0), -1)
    cv2.rectangle(image, (175, 90), (202, 103), (0, 0, 255), -1)
    cv2.rectangle(image, (310, 75), (335, 88), (0, 255, 0), -1)
    return image


def test_camera_detector_distinguishes_three_affiliations():
    candidates = detect_affiliated_candidates(
        _identity_test_image(), min_pixels=8, padding=3
    )
    assert {item.affiliation for item in candidates} == {
        AFFILIATION_FRIENDLY,
        AFFILIATION_HOSTILE,
        AFFILIATION_NEUTRAL,
    }


def test_camera_detector_rejects_small_color_noise():
    image = np.zeros((240, 400, 3), dtype=np.uint8)
    image[:] = (100, 100, 100)
    image[30, 30] = (0, 0, 255)
    image[110, 210] = (255, 0, 0)
    assert detect_affiliated_candidates(
        image, min_pixels=8, padding=3
    ) == []


def test_camera_detector_preserves_distant_two_pixel_identity_plate():
    image = np.full((135, 240, 3), (150, 150, 150), dtype=np.uint8)
    image[68:70, 160:162] = (0, 0, 255)
    candidates = detect_affiliated_candidates(
        image, min_pixels=3, padding=7, maximum_center_y_ratio=0.9
    )
    assert len(candidates) == 1
    assert candidates[0].affiliation == AFFILIATION_HOSTILE


def test_affiliation_requires_multiple_frames():
    filter_ = AffiliationTemporalFilter(confirmation_frames=3)
    assert filter_.update('one', AFFILIATION_HOSTILE, 0.9, 0.0)[0] == 0
    assert filter_.update('one', AFFILIATION_HOSTILE, 0.9, 0.1)[0] == 0
    assert filter_.update('one', AFFILIATION_HOSTILE, 0.9, 0.2)[0] == 2


def test_friendly_hostile_switch_is_stricter():
    filter_ = AffiliationTemporalFilter(
        history_size=8, confirmation_frames=2, switch_frames=4
    )
    filter_.update('one', AFFILIATION_FRIENDLY, 0.9, 0.0)
    assert filter_.update('one', AFFILIATION_FRIENDLY, 0.9, 0.1)[0] == 1
    for index in range(3):
        assert filter_.update(
            'one', AFFILIATION_HOSTILE, 0.95, 0.2 + index * 0.1
        )[0] == 1
    assert filter_.update('one', AFFILIATION_HOSTILE, 0.95, 0.5)[0] == 2


def test_affiliation_hold_and_timeout():
    filter_ = AffiliationTemporalFilter(
        confirmation_frames=1, hold_seconds=1.0, unknown_timeout=2.0
    )
    filter_.update('one', AFFILIATION_HOSTILE, 0.9, 0.0)
    assert filter_.mark_missing('one', 0.8)[0] == AFFILIATION_HOSTILE
    affiliation, confidence, reason = filter_.mark_missing('one', 2.1)
    assert affiliation == AFFILIATION_UNKNOWN
    assert confidence == 0.0
    assert reason == 'camera_timeout'


def test_projection_and_roi_are_camera_led():
    points = np.asarray(((10, 0, 0), (10, -1, 0), (10, 5, 0)))
    pixels, valid = project_camera_points(points, (100, 100, 50, 50))
    indices = roi_indices(pixels, (45, 45, 65, 55), available=valid)
    assert indices.tolist() == [0, 1]


def test_camera_projection_round_trip_uses_gazebo_axes():
    points = np.asarray(((12.0, -1.5, 0.8), (8.0, 2.0, -0.5)))
    intrinsics = (210.0, 208.0, 159.5, 89.5)
    pixels, valid = project_camera_points(points, intrinsics)
    restored = unproject_camera_pixels(pixels, points[:, 0], intrinsics)
    assert valid.tolist() == [True, True]
    assert np.allclose(restored, points)


def test_roi_available_mask_prevents_double_claim():
    pixels = np.asarray(((50, 50), (52, 50), (54, 50), (150, 50)))
    available = np.ones(4, dtype=bool)
    first = roi_indices(pixels, (40, 40, 60, 60), available=available)
    available[first] = False
    second = roi_indices(pixels, (45, 40, 65, 60), available=available)
    assert first.tolist() == [0, 1, 2]
    assert second.tolist() == []


def test_depth_filter_rejects_far_outlier():
    points = np.asarray(((10, 0, 0), (10.1, 0, 0), (10.2, 0, 0), (35, 0, 0)))
    keep = depth_filter(points, 0.0, 100.0, 0.8)
    assert keep.tolist() == [True, True, True, False]


def test_local_dbscan_separates_two_targets():
    left = np.random.default_rng(4).normal((0, 0, 1), 0.05, (20, 3))
    right = np.random.default_rng(5).normal((3, 0, 1), 0.05, (20, 3))
    labels = dbscan(np.vstack((left, right)), 0.25, 4)
    assert set(labels) == {0, 1}


def test_robust_bbox_ignores_extreme_points():
    rng = np.random.default_rng(7)
    body = rng.uniform((-3, -1, 0.2), (3, 1, 2), (600, 3))
    points = np.vstack((body, ((100, 100, 50),)))
    box = robust_oriented_bbox(points, 5, 95)
    assert 4.5 < max(box['dimensions'][:2]) < 7.0
    assert 1.4 < min(box['dimensions'][:2]) < 2.5
    assert box['dimensions'][2] < 2.2


def test_bbox_plausibility_uses_configured_limits():
    box = {'dimensions': np.asarray((6.0, 2.0, 1.5))}
    limits = {
        'minimum_length': 1.0, 'maximum_length': 10.0,
        'minimum_width': 0.5, 'maximum_width': 4.0,
        'minimum_height': 0.2, 'maximum_height': 4.0,
    }
    assert bbox_is_plausible(box, limits)
    box['dimensions'][2] = 8.0
    assert not bbox_is_plausible(box, limits)


def test_occluded_surface_gets_conservative_depth_away_from_sensor():
    box = {
        'center': np.asarray((10.0, 0.0, 1.0)),
        'dimensions': np.asarray((2.0, 0.001, 1.5)),
        'yaw': math.pi / 2.0,
    }
    completed = complete_occluded_bbox(box, (0.0, 0.0, 1.0), 0.6)
    assert completed['dimensions'][1] == 0.6
    assert completed['center'][0] > box['center'][0]
    assert completed['occlusion_completed']


def test_cluster_prediction_selects_nearest_supported_cluster():
    points = np.asarray(((0, 0, 0), (0.1, 0, 0), (5, 0, 0), (5.1, 0, 0)))
    labels = np.asarray((0, 0, 1, 1))
    indices, _ = select_cluster(points, labels, predicted_position=(5, 0, 0))
    assert indices.tolist() == [2, 3]


def test_bbox_smoothing_rejects_teleport():
    smoother = BboxSmoother(alpha=0.5, maximum_jump=2.0)
    base = {
        'center': np.asarray((0.0, 0.0, 1.0)),
        'dimensions': np.asarray((6.0, 2.0, 1.5)),
        'yaw': 0.0,
    }
    smoother.update('track', base)
    moved = dict(base, center=np.asarray((10.0, 0.0, 1.0)))
    result, accepted = smoother.update('track', moved)
    assert not accepted
    assert np.allclose(result['center'], base['center'])


def test_bbox_smoothing_accepts_persistent_relocation():
    smoother = BboxSmoother(
        alpha=0.5, maximum_jump=0.75, jump_confirmation_frames=3
    )
    base = {
        'center': np.asarray((0.0, 0.0, 1.0)),
        'dimensions': np.asarray((6.0, 2.0, 1.5)),
        'yaw': 0.0,
    }
    moved = dict(base, center=np.asarray((3.0, 0.0, 1.0)))
    smoother.update('track', base)
    assert not smoother.update('track', moved)[1]
    assert not smoother.update('track', moved)[1]
    result, accepted = smoother.update('track', moved)
    assert accepted
    assert np.allclose(result['center'], moved['center'])
