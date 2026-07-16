from builtin_interfaces.msg import Time
from uav_usv_interfaces.msg import TrackedObject
from uav_usv_interfaces.msg import TrackedObjectArray

from lv_dot_observation_adapter import adapt_dynamic_message


def test_adapter_preserves_contract_and_adds_lidar_source():
    message = TrackedObjectArray()
    message.header.frame_id = 'map'
    tracked = TrackedObject()
    tracked.track_id = 'dynamic_track_001'
    tracked.confidence = 1.25
    tracked.pose.covariance[0] = 0.42
    message.objects.append(tracked)
    fallback = Time(sec=42, nanosec=123)

    output, statistics = adapt_dynamic_message(message, fallback)

    assert output.header.frame_id == 'map'
    assert output.header.stamp == fallback
    assert len(output.objects) == 1
    adapted = output.objects[0]
    assert adapted.track_id == 'dynamic_track_001'
    assert adapted.source_mask & TrackedObject.SOURCE_LIDAR
    assert adapted.classification == TrackedObject.CLASS_UNKNOWN
    assert adapted.confidence == 1.0
    assert adapted.pose.covariance[0] == 0.42
    assert adapted.first_seen == fallback
    assert adapted.last_update == fallback
    assert any(adapted.uuid.uuid)
    assert statistics['input_count'] == 1
    assert statistics['output_count'] == 1
    assert statistics['clamped_confidence'] == 1


def test_adapter_drops_track_without_stable_id():
    message = TrackedObjectArray()
    message.objects.append(TrackedObject())

    output, statistics = adapt_dynamic_message(message, Time(sec=1))

    assert not output.objects
    assert statistics['dropped_missing_track_id'] == 1
