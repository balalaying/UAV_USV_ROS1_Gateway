"""Small node API adapter used by the repository's ROS 1 Qt nodes.

It intentionally covers only the APIs used by fleet_base_station and the Qt
client.  Transport and message types remain native rospy objects.
"""

import threading

import rospy


class ExternalShutdownException(Exception):
    pass


class _PolicyValue:
    def __init__(self, value):
        self.value = value


class ReliabilityPolicy:
    RELIABLE = _PolicyValue('reliable')
    BEST_EFFORT = _PolicyValue('best_effort')


class DurabilityPolicy:
    VOLATILE = _PolicyValue('volatile')
    TRANSIENT_LOCAL = _PolicyValue('transient_local')


class QoSProfile:
    def __init__(self, depth=10, **_kwargs):
        self.depth = int(depth)
        self.reliability = ReliabilityPolicy.RELIABLE
        self.durability = DurabilityPolicy.VOLATILE


qos_profile_sensor_data = QoSProfile(depth=1)


class ReentrantCallbackGroup:
    pass


class MutuallyExclusiveCallbackGroup:
    pass


class _Parameter:
    def __init__(self, value):
        self.value = value


class _ClockTime:
    def __init__(self, value=None):
        self._value = value or rospy.Time.now()

    @property
    def nanoseconds(self):
        return int(self._value.to_nsec())

    def to_msg(self):
        return self._value


class _Clock:
    @staticmethod
    def now():
        return _ClockTime()


class Time:
    def __new__(cls, seconds=0, nanoseconds=0, **_kwargs):
        return rospy.Time(int(seconds), int(nanoseconds))

    @staticmethod
    def from_msg(value):
        return value


class Duration:
    def __init__(self, seconds=0.0):
        self._value = rospy.Duration.from_sec(float(seconds))

    def to_msg(self):
        return self._value


class _Logger:
    def __init__(self, name):
        self.name = name

    @staticmethod
    def _throttle(kwargs):
        return float(kwargs.pop('throttle_duration_sec', 0.0) or 0.0)

    def _log(self, plain, throttled, message, kwargs):
        period = self._throttle(kwargs)
        if period > 0.0:
            throttled(period, message)
        else:
            plain(message)

    def info(self, message, **kwargs):
        self._log(rospy.loginfo, rospy.loginfo_throttle, message, kwargs)

    def warn(self, message, **kwargs):
        self._log(rospy.logwarn, rospy.logwarn_throttle, message, kwargs)

    warning = warn

    def error(self, message, **kwargs):
        self._log(rospy.logerr, rospy.logerr_throttle, message, kwargs)

    def debug(self, message, **kwargs):
        self._log(rospy.logdebug, rospy.logdebug_throttle, message, kwargs)


class _Publisher:
    def __init__(self, publisher):
        self._publisher = publisher

    def publish(self, message):
        self._publisher.publish(message)

    def get_subscription_count(self):
        return self._publisher.get_num_connections()

    def unregister(self):
        self._publisher.unregister()


class Node:
    def __init__(self, name):
        self._name = name
        self._logger = _Logger(name)
        self._parameters = {}
        self._publishers = []
        self._subscribers = []
        self._timers = []

    def declare_parameter(self, name, default_value=None):
        key = '~' + name
        if not rospy.has_param(key):
            rospy.set_param(key, default_value)
        self._parameters[name] = key
        return _Parameter(rospy.get_param(key))

    def get_parameter(self, name):
        key = self._parameters.get(name, '~' + name)
        return _Parameter(rospy.get_param(key))

    def create_publisher(self, message_type, topic, qos):
        depth = qos.depth if isinstance(qos, QoSProfile) else int(qos)
        latch = (
            isinstance(qos, QoSProfile)
            and qos.durability is DurabilityPolicy.TRANSIENT_LOCAL
        )
        publisher = _Publisher(rospy.Publisher(
            topic, message_type, queue_size=max(1, depth), latch=latch
        ))
        self._publishers.append(publisher)
        return publisher

    def create_subscription(
        self, message_type, topic, callback, qos, callback_group=None
    ):
        del callback_group
        depth = qos.depth if isinstance(qos, QoSProfile) else int(qos)
        subscriber = rospy.Subscriber(
            topic, message_type, callback, queue_size=max(1, depth)
        )
        self._subscribers.append(subscriber)
        return subscriber

    def create_timer(self, period, callback, callback_group=None):
        del callback_group
        timer = rospy.Timer(
            rospy.Duration.from_sec(float(period)), lambda _event: callback()
        )
        self._timers.append(timer)
        return timer

    def get_clock(self):
        return _Clock()

    def get_logger(self):
        return self._logger

    def destroy_node(self):
        for timer in self._timers:
            timer.shutdown()
        for subscriber in self._subscribers:
            subscriber.unregister()
        for publisher in self._publishers:
            publisher.unregister()


class MultiThreadedExecutor:
    def __init__(self, num_threads=1):
        self.num_threads = num_threads
        self._shutdown = threading.Event()

    def add_node(self, _node):
        pass

    def spin(self):
        rospy.spin()

    def shutdown(self, timeout_sec=None):
        del timeout_sec
        self._shutdown.set()


def init(args=None, name=None):
    del args
    if not rospy.core.is_initialized():
        rospy.init_node(name or 'uav_usv_node', disable_signals=True)


def spin(_node):
    rospy.spin()


def ok():
    return not rospy.is_shutdown()


def shutdown():
    if not rospy.is_shutdown():
        rospy.signal_shutdown('node requested shutdown')
