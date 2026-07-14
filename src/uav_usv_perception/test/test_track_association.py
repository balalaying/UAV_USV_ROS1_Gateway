from dataclasses import dataclass

import pytest

from track_association import fused_confidence
from track_association import nearest_track
from track_association import planar_distance
from track_association import stable_uuid_bytes


@dataclass
class Track:
    position: tuple


def test_planar_distance():
    assert planar_distance((0.0, 0.0), (3.0, 4.0)) == pytest.approx(5.0)


def test_fused_confidence():
    assert fused_confidence([0.8]) == pytest.approx(0.8)
    assert fused_confidence([0.8, 0.5]) == pytest.approx(0.9)
    assert fused_confidence([]) == 0.0


def test_stable_uuid():
    first = stable_uuid_bytes('target_vessel')
    assert first == stable_uuid_bytes('target_vessel')
    assert first != stable_uuid_bytes('another_target')
    assert len(first) == 16


def test_nearest_track_respects_gate_and_exclusion():
    tracks = {
        'near': Track((2.0, 0.0, 0.0)),
        'far': Track((12.0, 0.0, 0.0)),
    }
    assert nearest_track((0.0, 0.0, 0.0), tracks, 5.0) == 'near'
    assert nearest_track(
        (0.0, 0.0, 0.0), tracks, 20.0, excluded={'near'}
    ) == 'far'
    assert nearest_track((0.0, 0.0, 0.0), tracks, 1.0) is None
