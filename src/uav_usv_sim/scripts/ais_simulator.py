#!/usr/bin/env python3

from collections import deque
import hashlib
import math
import random
import time

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzTransportNode
import rospy
from uuid_msgs.msg import UniqueID
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from uav_usv_interfaces.msg import AisContact
from uav_usv_interfaces.msg import AisContactArray
from uav_usv_interfaces.msg import TrackedObject
from uav_usv_interfaces.msg import TrackedObjectArray


EARTH_RADIUS_M = 6378137.0


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def enu_yaw_to_navigation_angle(yaw):
    return (0.5 * math.pi - yaw) % (2.0 * math.pi)


class AisSimulator:
    """Produces delayed and noisy AIS plus the common tracked-object format."""

    def __init__(self):
        self.pose_topic = rospy.get_param(
            '~pose_topic',
            '/world/default/pose/info',
        )
        self.target_name = rospy.get_param(
            '~target_name',
            'target_vessel',
        )
        self.map_frame_id = rospy.get_param(
            '~map_frame_id',
            'map',
        )
        self.ais_topic = rospy.get_param(
            '~ais_topic',
            '/maritime/ais/contacts',
        )
        self.tracks_topic = rospy.get_param(
            '~tracks_topic',
            '/maritime/tracks/ais',
        )
        self.marker_topic = rospy.get_param(
            '~marker_topic',
            '/maritime/ais/markers',
        )

        self.mmsi = int(rospy.get_param('~mmsi', 413000001))
        self.vessel_name = rospy.get_param(
            '~vessel_name',
            'SIM_TARGET_01',
        )

        self.update_period = max(
            0.1,
            float(rospy.get_param('~update_period', 2.0)),
        )
        self.latency = max(
            0.0,
            float(rospy.get_param('~latency', 0.5)),
        )
        self.dropout_probability = min(
            1.0,
            max(
                0.0,
                float(rospy.get_param('~dropout_probability', 0.03)),
            ),
        )
        self.position_noise_std = max(
            0.0,
            float(rospy.get_param('~position_noise_std', 0.8)),
        )
        self.speed_noise_std = max(
            0.0,
            float(rospy.get_param('~speed_noise_std', 0.05)),
        )
        self.heading_noise_std = max(
            0.0,
            float(rospy.get_param('~heading_noise_std', 0.015)),
        )

        self.origin_latitude = float(
            rospy.get_param(
                '~origin_latitude',
                47.397971057728974,
            )
        )
        self.origin_longitude = float(
            rospy.get_param(
                '~origin_longitude',
                8.546163739800146,
            )
        )
        self.origin_altitude = float(
            rospy.get_param('~origin_altitude', 0.0)
        )

        self.dimensions = (
            float(rospy.get_param('~vessel_length', 7.0)),
            float(rospy.get_param('~vessel_width', 2.6)),
            float(rospy.get_param('~vessel_height', 3.4)),
        )

        self.random = random.Random(
            int(rospy.get_param('~random_seed', 7))
        )

        self.ais_pub = rospy.Publisher(
            self.ais_topic,
            AisContactArray,
            queue_size=10,
        )
        self.track_pub = rospy.Publisher(
            self.tracks_topic,
            TrackedObjectArray,
            queue_size=10,
        )
        self.marker_pub = rospy.Publisher(
            self.marker_topic,
            MarkerArray,
            queue_size=1,
            latch=True,
        )

        self.gz_node = GzTransportNode()
        self.gz_node.subscribe(
            Pose_V,
            self.pose_topic,
            self._on_pose,
        )

        self.pose = None
        self.previous_pose = None
        self.previous_time = None
        self.velocity = (0.0, 0.0)
        self.first_seen = None
        self.pending = deque()

        self.sample_timer = rospy.Timer(
            rospy.Duration(self.update_period),
            self._sample,
        )
        self.delivery_timer = rospy.Timer(
            rospy.Duration(0.05),
            self._deliver,
        )

        digest = hashlib.sha256(
            ('ais:%d:%s' % (self.mmsi, self.target_name)).encode('utf-8')
        ).digest()

        self.uuid = UniqueID()
        self.uuid.uuid = list(digest[:16])

        rospy.loginfo(
            'AIS simulator tracking %s as MMSI %d, period=%.1f s, '
            'latency=%.1f s.',
            self.target_name,
            self.mmsi,
            self.update_period,
            self.latency,
        )

    def _on_pose(self, msg):
        now = time.monotonic()

        for pose in msg.pose:
            if pose.name != self.target_name:
                continue

            if self.pose is not None and self.previous_time is not None:
                dt = now - self.previous_time

                if dt > 1e-3:
                    self.velocity = (
                        (pose.position.x - self.pose.position.x) / dt,
                        (pose.position.y - self.pose.position.y) / dt,
                    )

            self.previous_pose = self.pose
            self.pose = pose
            self.previous_time = now
            return

    def _sample(self, _event):
        if self.pose is None:
            return

        if self.random.random() < self.dropout_probability:
            rospy.logdebug('Simulated AIS packet drop')
            return

        stamp = rospy.Time.now()

        if self.first_seen is None:
            self.first_seen = stamp

        noisy_x = self.pose.position.x + self.random.gauss(
            0.0,
            self.position_noise_std,
        )
        noisy_y = self.pose.position.y + self.random.gauss(
            0.0,
            self.position_noise_std,
        )

        vx = self.velocity[0] + self.random.gauss(
            0.0,
            self.speed_noise_std,
        )
        vy = self.velocity[1] + self.random.gauss(
            0.0,
            self.speed_noise_std,
        )

        yaw = yaw_from_quaternion(self.pose.orientation)
        noisy_yaw = yaw + self.random.gauss(
            0.0,
            self.heading_noise_std,
        )

        speed = math.hypot(vx, vy)
        cog_enu = math.atan2(vy, vx) if speed > 0.05 else noisy_yaw

        contact_array = AisContactArray()
        contact_array.header.stamp = stamp
        contact_array.header.frame_id = self.map_frame_id

        contact = AisContact()
        contact.header = contact_array.header
        contact.mmsi = self.mmsi
        contact.vessel_name = self.vessel_name

        (
            contact.position.latitude,
            contact.position.longitude,
        ) = self._enu_to_geo(noisy_x, noisy_y)

        contact.position.altitude = (
            self.origin_altitude + self.pose.position.z
        )
        contact.sog_mps = float(max(0.0, speed))
        contact.cog_rad = float(
            enu_yaw_to_navigation_angle(cog_enu)
        )
        contact.heading_rad = float(
            enu_yaw_to_navigation_angle(noisy_yaw)
        )
        contact.navigation_status = AisContact.NAV_STATUS_UNDER_WAY

        (
            contact.dimensions.x,
            contact.dimensions.y,
            contact.dimensions.z,
        ) = self.dimensions

        contact.position_accuracy_m = float(
            self.position_noise_std
        )

        contact_array.contacts.append(contact)

        tracks = TrackedObjectArray()
        tracks.header = contact_array.header

        track = TrackedObject()
        track.uuid = self.uuid
        track.track_id = 'ais_%d' % self.mmsi
        track.first_seen = self.first_seen
        track.last_update = stamp
        track.source_mask = TrackedObject.SOURCE_AIS
        track.classification = TrackedObject.CLASS_VESSEL

        track.pose.pose.position.x = noisy_x
        track.pose.pose.position.y = noisy_y
        track.pose.pose.position.z = self.pose.position.z

        qx, qy, qz, qw = quaternion_from_yaw(noisy_yaw)

        track.pose.pose.orientation.x = qx
        track.pose.pose.orientation.y = qy
        track.pose.pose.orientation.z = qz
        track.pose.pose.orientation.w = qw

        variance = self.position_noise_std ** 2

        track.pose.covariance[0] = variance
        track.pose.covariance[7] = variance
        track.pose.covariance[14] = max(0.04, variance)
        track.pose.covariance[35] = self.heading_noise_std ** 2

        track.twist.twist.linear.x = vx
        track.twist.twist.linear.y = vy

        speed_variance = self.speed_noise_std ** 2
        track.twist.covariance[0] = speed_variance
        track.twist.covariance[7] = speed_variance

        (
            track.dimensions.x,
            track.dimensions.y,
            track.dimensions.z,
        ) = self.dimensions

        track.confidence = 0.95
        track.mmsi = self.mmsi

        tracks.objects.append(track)

        self.pending.append(
            (
                time.monotonic() + self.latency,
                contact_array,
                tracks,
            )
        )

    def _deliver(self, _event):
        now = time.monotonic()

        while self.pending and self.pending[0][0] <= now:
            _, contacts, tracks = self.pending.popleft()

            self.ais_pub.publish(contacts)
            self.track_pub.publish(tracks)
            self._publish_markers(tracks)

    def _enu_to_geo(self, east, north):
        latitude = (
            self.origin_latitude
            + math.degrees(north / EARTH_RADIUS_M)
        )

        longitude = (
            self.origin_longitude
            + math.degrees(
                east
                / (
                    EARTH_RADIUS_M
                    * max(
                        1e-6,
                        math.cos(
                            math.radians(self.origin_latitude)
                        ),
                    )
                )
            )
        )

        return latitude, longitude

    def _publish_markers(self, tracks):
        markers = MarkerArray()

        for index, track in enumerate(tracks.objects):
            vessel = Marker()
            vessel.header = tracks.header
            vessel.ns = 'ais_vessels'
            vessel.id = index * 2
            vessel.type = Marker.CUBE
            vessel.action = Marker.ADD
            vessel.pose = track.pose.pose
            vessel.pose.position.z += 0.7
            vessel.scale = track.dimensions

            vessel.color.r = 0.1
            vessel.color.g = 0.8
            vessel.color.b = 1.0
            vessel.color.a = 0.55

            markers.markers.append(vessel)

            label = Marker()
            label.header = tracks.header
            label.ns = 'ais_labels'
            label.id = index * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD

            label.pose.position.x = track.pose.pose.position.x
            label.pose.position.y = track.pose.pose.position.y
            label.pose.position.z = 4.5

            label.scale.z = 1.2

            label.color.r = 0.85
            label.color.g = 0.95
            label.color.b = 1.0
            label.color.a = 1.0

            label.text = '%s\nMMSI %d' % (
                self.vessel_name,
                self.mmsi,
            )

            markers.markers.append(label)

        self.marker_pub.publish(markers)


def main():
    rospy.init_node('ais_simulator')

    AisSimulator()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
