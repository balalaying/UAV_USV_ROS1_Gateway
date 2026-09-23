"""ROS 1 / ROS 2 compatible accessors for time-like message fields.

ROS 1 ``genpy.Time`` and ``genpy.Duration`` expose ``secs`` / ``nsecs``.
ROS 2 message types expose ``sec`` / ``nanosec``.  The helpers in this
module keep application code independent from that spelling difference and
do not alter the serialized ROS message layout.
"""


def time_parts(value):
    """Return ``(seconds, nanoseconds)`` from a ROS time-like object."""
    seconds = getattr(value, 'sec', None)
    if seconds is None:
        seconds = getattr(value, 'secs', 0)
    nanoseconds = getattr(value, 'nanosec', None)
    if nanoseconds is None:
        nanoseconds = getattr(value, 'nsecs', 0)
    return int(seconds), int(nanoseconds)


def time_to_seconds(value):
    """Convert a ROS time-like object to floating-point seconds."""
    seconds, nanoseconds = time_parts(value)
    return float(seconds) + float(nanoseconds) * 1.0e-9


def time_to_nanoseconds(value):
    """Convert a ROS time-like object to integer nanoseconds."""
    seconds, nanoseconds = time_parts(value)
    return seconds * 1000000000 + nanoseconds


def time_is_zero(value):
    """Return whether both components of a ROS time-like object are zero."""
    return time_parts(value) == (0, 0)


def time_dict(value):
    """Return the transport-neutral JSON representation used by the repo."""
    seconds, nanoseconds = time_parts(value)
    return {
        'sec': seconds,
        'nanosec': nanoseconds,
        'seconds': float(seconds) + float(nanoseconds) * 1.0e-9,
    }


def set_time_fields(value, seconds=0, nanoseconds=0):
    """Set either ROS 1 or ROS 2 component names on a time-like object."""
    seconds = int(seconds)
    nanoseconds = int(nanoseconds)
    if hasattr(value, 'sec'):
        value.sec = seconds
    else:
        value.secs = seconds
    if hasattr(value, 'nanosec'):
        value.nanosec = nanoseconds
    else:
        value.nsecs = nanoseconds
    return value


__all__ = [
    'set_time_fields', 'time_dict', 'time_is_zero', 'time_parts',
    'time_to_nanoseconds', 'time_to_seconds',
]
