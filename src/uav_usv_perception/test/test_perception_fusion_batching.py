from types import SimpleNamespace

from perception_fusion_node import ObservationBatch
from perception_fusion_node import partition_ready_groups
from perception_fusion_node import select_synchronized_batches


def _candidate(topic, received_at):
    return SimpleNamespace(topic=topic, received_at=received_at)


def test_complete_multisensor_group_is_ready_immediately():
    group = [
        _candidate('ground_truth', 10.0),
        _candidate('lidar', 10.01),
        _candidate('camera', 10.02),
    ]

    ready, waiting = partition_ready_groups([group], 3, 0.2, 10.03)

    assert ready == [group]
    assert waiting == []


def test_incomplete_group_waits_then_expires_to_existing_fusion_path():
    group = [_candidate('ground_truth', 10.0)]

    ready, waiting = partition_ready_groups([group], 3, 0.2, 10.1)
    assert ready == []
    assert waiting == [group]

    ready, waiting = partition_ready_groups([group], 3, 0.2, 10.21)
    assert ready == [group]
    assert waiting == []


def test_zero_wait_preserves_legacy_behavior():
    group = [_candidate('ground_truth', 10.0)]

    ready, waiting = partition_ready_groups([group], 3, 0.0, 10.0)

    assert ready == [group]
    assert waiting == []


def _batch(topic, observed_at, count=1):
    return ObservationBatch(
        topic=topic,
        observed_at=observed_at,
        received_at=20.0,
        candidates=[object()] * count,
    )


def test_history_selection_uses_slowest_latest_source_as_watermark():
    histories = {
        'ground_truth': [
            _batch('ground_truth', 10.0),
            _batch('ground_truth', 10.4),
        ],
        'lidar': [_batch('lidar', 10.2)],
        'camera': [
            _batch('camera', 10.19),
            _batch('camera', 10.3),
        ],
    }

    selected = select_synchronized_batches(
        histories,
        ('ground_truth', 'lidar', 'camera'),
        0.25,
    )

    assert [batch.observed_at for batch in selected] == [10.0, 10.2, 10.19]


def test_history_selection_rejects_temporally_incoherent_sources():
    histories = {
        'ground_truth': [_batch('ground_truth', 10.0)],
        'lidar': [_batch('lidar', 10.1)],
        'camera': [_batch('camera', 11.0)],
    }

    selected = select_synchronized_batches(
        histories,
        ('ground_truth', 'lidar', 'camera'),
        0.25,
    )

    assert selected == []


def test_history_selection_ignores_empty_detection_batches():
    histories = {
        'ground_truth': [_batch('ground_truth', 10.0)],
        'lidar': [_batch('lidar', 10.0)],
        'camera': [
            _batch('camera', 9.9),
            _batch('camera', 10.0, count=0),
        ],
    }

    selected = select_synchronized_batches(
        histories,
        ('ground_truth', 'lidar', 'camera'),
        0.25,
    )

    assert [batch.observed_at for batch in selected] == [10.0, 10.0, 9.9]
