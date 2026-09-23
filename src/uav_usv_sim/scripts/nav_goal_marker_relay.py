#!/usr/bin/env python3
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
from visualization_msgs.msg import Marker, MarkerArray


class NavGoalMarkerRelay:
    def __init__(self):
        self.goal_topic = rospy.get_param('~goal_topic', '/goal_pose')
        self.marker_topic = rospy.get_param(
            '~marker_topic',
            '/boat/nav_goal_marker',
        )
        self.action_name = rospy.get_param('~action_name', 'move_base')
        self.default_frame_id = rospy.get_param(
            '~default_frame_id',
            'map',
        )

        self.marker_pub = rospy.Publisher(
            self.marker_topic,
            MarkerArray,
            queue_size=1,
            latch=True,
        )

        self.goal_sub = rospy.Subscriber(
            self.goal_topic,
            PoseStamped,
            self._on_goal,
            queue_size=10,
        )

        self.action_client = actionlib.SimpleActionClient(
            self.action_name,
            MoveBaseAction,
        )

        self.pending_goal = None
        self.retry_timer = rospy.Timer(
            rospy.Duration(0.5),
            self._send_pending_goal,
        )

    def _on_goal(self, msg):
        if not msg.header.frame_id:
            msg.header.frame_id = self.default_frame_id

        if msg.header.stamp == rospy.Time():
            msg.header.stamp = rospy.Time.now()

        self.pending_goal = msg
        self._publish_marker(msg)
        self._send_pending_goal()

    def _send_pending_goal(self, _event=None):
        if self.pending_goal is None:
            return

        if not self.action_client.wait_for_server(rospy.Duration(0.01)):
            return

        goal = MoveBaseGoal()
        goal.target_pose = self.pending_goal
        self.pending_goal = None
        self.action_client.send_goal(goal)

    def _publish_marker(self, goal):
        markers = MarkerArray()

        base = Marker()
        base.header = goal.header
        base.ns = 'navigation_goal'
        base.id = 0
        base.type = Marker.CYLINDER
        base.action = Marker.ADD
        base.pose = goal.pose
        base.pose.position.z = 0.18
        base.scale.x = 1.6
        base.scale.y = 1.6
        base.scale.z = 0.35
        base.color.r = 1.0
        base.color.g = 0.22
        base.color.b = 0.08
        base.color.a = 0.95
        base.lifetime = rospy.Duration(0.0)
        markers.markers.append(base)

        arrow = Marker()
        arrow.header = goal.header
        arrow.ns = 'navigation_goal'
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose = goal.pose
        arrow.pose.position.z = 0.55
        arrow.scale.x = 3.2
        arrow.scale.y = 0.35
        arrow.scale.z = 0.45
        arrow.color.r = 1.0
        arrow.color.g = 0.75
        arrow.color.b = 0.08
        arrow.color.a = 1.0
        arrow.lifetime = rospy.Duration(0.0)
        markers.markers.append(arrow)

        label = Marker()
        label.header = goal.header
        label.ns = 'navigation_goal'
        label.id = 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = goal.pose.position.x
        label.pose.position.y = goal.pose.position.y
        label.pose.position.z = 2.2
        label.pose.orientation.w = 1.0
        label.scale.z = 1.1
        label.color.r = 1.0
        label.color.g = 0.9
        label.color.b = 0.55
        label.color.a = 1.0
        label.text = 'Navigation Goal'
        label.lifetime = rospy.Duration(0.0)
        markers.markers.append(label)

        self.marker_pub.publish(markers)


def main():
    rospy.init_node('nav_goal_marker_relay')
    NavGoalMarkerRelay()
    rospy.spin()


if __name__ == '__main__':
    main()
