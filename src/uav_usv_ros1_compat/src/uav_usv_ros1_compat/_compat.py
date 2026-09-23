"""Small node facade backed by native rospy transport and messages."""

import inspect
import threading

import genpy
import rospy

from uav_usv_ros1_compat.time_fields import time_parts


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


class HistoryPolicy:
    KEEP_LAST = _PolicyValue('keep_last')
    KEEP_ALL = _PolicyValue('keep_all')


class QoSProfile:
    def __init__(
        self,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        **_kwargs
    ):
        self.depth = int(depth)
        self.reliability = reliability
        self.durability = durability
        self.history = history


qos_profile_sensor_data = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
)


class ReentrantCallbackGroup:
    pass


class MutuallyExclusiveCallbackGroup:
    pass


class Parameter:
    class Type:
        NOT_SET = 0
        BOOL = 1
        INTEGER = 2
        DOUBLE = 3
        STRING = 4
        BYTE_ARRAY = 5
        BOOL_ARRAY = 6
        INTEGER_ARRAY = 7
        DOUBLE_ARRAY = 8
        STRING_ARRAY = 9

    def __init__(self, name='', type_=None, value=None):
        self.name = str(name)
        self.type_ = type_
        self.value = value


def _duration_nanoseconds(value):
    return int(value.to_nsec())


def _time_nanoseconds(value):
    return int(value.to_nsec())


def _component_property(component):
    return property(
        lambda self: getattr(self, component),
        lambda self, value: setattr(self, component, int(value)),
    )


def _install_time_aliases(value_type, nanoseconds_reader):
    """Add writable ROS 2 spellings to a ROS 1 time-like class."""
    if not hasattr(value_type, 'nanoseconds'):
        value_type.nanoseconds = property(nanoseconds_reader)
    if not hasattr(value_type, 'sec'):
        value_type.sec = _component_property('secs')
    if not hasattr(value_type, 'nanosec'):
        value_type.nanosec = _component_property('nsecs')
    if not hasattr(value_type, 'to_msg'):
        value_type.to_msg = lambda self: self


# Generated ROS 1 messages hold ``genpy`` base-class instances, which are not
# guaranteed to be instances of the public ``rospy`` subclasses.  Patch both
# layers so reads and writes remain compatible while the on-wire ROS 1 type is
# unchanged.
for _type in {genpy.Duration, rospy.Duration}:
    _install_time_aliases(_type, _duration_nanoseconds)
for _type in {genpy.Time, rospy.Time}:
    _install_time_aliases(_type, _time_nanoseconds)


class Time(rospy.Time):
    def __init__(
        self, seconds=0, nanoseconds=0, sec=None, nanosec=None, **_kwargs
    ):
        super().__init__(
            secs=int(seconds if sec is None else sec),
            nsecs=int(nanoseconds if nanosec is None else nanosec),
        )

    @classmethod
    def from_msg(cls, value):
        seconds, nanoseconds = time_parts(value)
        return cls(seconds=seconds, nanoseconds=nanoseconds)

    def to_msg(self):
        return rospy.Time(self.secs, self.nsecs)


class Duration(rospy.Duration):
    def __init__(
        self, seconds=0.0, nanoseconds=0, sec=None, nanosec=None, **_kwargs
    ):
        if sec is not None or nanosec is not None:
            secs = int(0 if sec is None else sec)
            nsecs = int(0 if nanosec is None else nanosec)
        else:
            total = float(seconds) + int(nanoseconds) * 1.0e-9
            secs = int(total)
            nsecs = int(round((total - secs) * 1.0e9))
        super().__init__(secs=secs, nsecs=nsecs)

    def to_msg(self):
        return rospy.Duration(self.secs, self.nsecs)


class _Clock:
    @staticmethod
    def now():
        value = rospy.Time.now()
        return Time(seconds=value.secs, nanoseconds=value.nsecs)


class _Logger:
    @staticmethod
    def _throttle(kwargs):
        return float(kwargs.pop('throttle_duration_sec', 0.0) or 0.0)

    @staticmethod
    def _log(plain, throttled, message, kwargs):
        period = _Logger._throttle(kwargs)
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


class _ServiceClient:
    def __init__(self, service_type, name):
        self.service_type = service_type
        self.name = name
        self.proxy = rospy.ServiceProxy(name, service_type)

    def wait_for_service(self, timeout_sec=None):
        try:
            rospy.wait_for_service(self.name, timeout=timeout_sec)
            return True
        except rospy.ROSException:
            return False

    def service_is_ready(self):
        return self.wait_for_service(0.001)

    def call_async(self, request):
        future = _Future()
        try:
            future.set_result(self.proxy(request))
        except Exception as error:
            future.set_exception(error)
        return future


class _Future:
    def __init__(self):
        self._result = None
        self._exception = None
        self._callbacks = []
        self._done = False

    def set_result(self, value):
        self._result = value
        self._done = True
        for callback in list(self._callbacks):
            callback(self)

    def set_exception(self, error):
        self._exception = error
        self._done = True
        for callback in list(self._callbacks):
            callback(self)

    def result(self):
        if self._exception is not None:
            raise self._exception
        return self._result

    def done(self):
        return self._done

    def add_done_callback(self, callback):
        self._callbacks.append(callback)
        if self._done:
            callback(self)


class Node:
    def __init__(self, name):
        if not rospy.core.is_initialized():
            rospy.init_node(name, disable_signals=True)
        self._name = name
        self._logger = _Logger()
        self._parameters = {}
        self._publishers = []
        self._subscribers = []
        self._timers = []
        self._services = []
        self._parameter_callbacks = []

    def get_name(self):
        return rospy.get_name().rsplit('/', 1)[-1]

    def get_namespace(self):
        name = rospy.get_name()
        return name.rsplit('/', 1)[0] or '/'

    def declare_parameter(self, name, default_value=None):
        key = '~' + name
        if not rospy.has_param(key):
            rospy.set_param(key, default_value)
        self._parameters[name] = key
        return Parameter(name=name, value=rospy.get_param(key))

    def get_parameter(self, name):
        key = self._parameters.get(name, '~' + name)
        return Parameter(name=name, value=rospy.get_param(key))

    def set_parameters(self, parameters):
        for callback in self._parameter_callbacks:
            result = callback(parameters)
            if not getattr(result, 'successful', False):
                return [result]
        for parameter in parameters:
            rospy.set_param('~' + parameter.name, parameter.value)
        return []

    def add_on_set_parameters_callback(self, callback):
        self._parameter_callbacks.append(callback)
        return callback

    @staticmethod
    def _depth(qos):
        return max(1, qos.depth if isinstance(qos, QoSProfile) else int(qos))

    def create_publisher(self, message_type, topic, qos):
        latch = (
            isinstance(qos, QoSProfile)
            and qos.durability is DurabilityPolicy.TRANSIENT_LOCAL
        )
        publisher = _Publisher(rospy.Publisher(
            topic, message_type, queue_size=self._depth(qos), latch=latch
        ))
        self._publishers.append(publisher)
        return publisher

    def create_subscription(
        self, message_type, topic, callback, qos, callback_group=None
    ):
        del callback_group
        subscriber = rospy.Subscriber(
            topic, message_type, callback, queue_size=self._depth(qos)
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

    def create_service(self, service_type, name, callback):
        parameters = len(inspect.signature(callback).parameters)

        def invoke(request):
            if parameters >= 2:
                response = service_type._response_class()
                return callback(request, response)
            return callback(request)

        service = rospy.Service(name, service_type, invoke)
        self._services.append(service)
        return service

    def create_client(self, service_type, name, callback_group=None):
        del callback_group
        return _ServiceClient(service_type, name)

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
        for service in self._services:
            service.shutdown()


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


class SingleThreadedExecutor(MultiThreadedExecutor):
    pass


def init(args=None, name=None):
    del args
    if not rospy.core.is_initialized():
        rospy.init_node(name or 'uav_usv_node', disable_signals=True)


def spin(_node):
    rospy.spin()


def spin_once(_node, timeout_sec=None):
    rospy.sleep(0.001 if timeout_sec is None else max(0.0, timeout_sec))


def spin_until_future_complete(_node, future, timeout_sec=None):
    deadline = None if timeout_sec is None else rospy.get_time() + timeout_sec
    rate = rospy.Rate(200)
    while not future.done() and not rospy.is_shutdown():
        if deadline is not None and rospy.get_time() >= deadline:
            break
        rate.sleep()
    return future


def ok():
    return not rospy.is_shutdown()


def shutdown():
    if not rospy.is_shutdown():
        rospy.signal_shutdown('node requested shutdown')
