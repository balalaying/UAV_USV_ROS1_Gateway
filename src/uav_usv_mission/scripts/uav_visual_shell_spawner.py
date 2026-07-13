#!/usr/bin/env python3
"""Spawn visual-only UAV shells and annotate their live capture roles."""

import math
import threading
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.marker_pb2 import Marker as GzMarker
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.visual_pb2 import Visual
from gz.transport13 import Node as GzNode
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from uav_usv_interfaces.msg import CaptureAssignmentArray
from uav_usv_interfaces.msg import VehicleState


class UavVisualShellSpawner(Node):
    COLORS = (
        (0.08, 0.45, 0.95, 1.0),
        (0.95, 0.30, 0.12, 1.0),
        (0.12, 0.72, 0.30, 1.0),
        (0.78, 0.22, 0.82, 1.0),
    )

    SEGMENTS = {
        0: 'abcdef',
        1: 'bc',
        2: 'abdeg',
        3: 'abcdg',
        4: 'bcfg',
        5: 'acdfg',
        6: 'acdefg',
        7: 'abc',
        8: 'abcdefg',
        9: 'abcdfg',
    }

    SEGMENT_POSES = {
        'a': (0.0, 0.13, 0.0, 0.22, 0.04),
        'b': (0.11, 0.065, 0.0, 0.04, 0.17),
        'c': (0.11, -0.065, 0.0, 0.04, 0.17),
        'd': (0.0, -0.13, 0.0, 0.22, 0.04),
        'e': (-0.11, -0.065, 0.0, 0.04, 0.17),
        'f': (-0.11, 0.065, 0.0, 0.04, 0.17),
        'g': (0.0, 0.0, 0.0, 0.22, 0.04),
    }

    def __init__(self):
        super().__init__('uav_visual_shell_spawner')
        self.declare_parameter(
            'uav_ids', ['uav_01', 'uav_02', 'uav_03', 'uav_04']
        )
        self.declare_parameter('world_name', 'fleet_dynamic_capture')
        self.declare_parameter('uav_visual_scale', 12.0)
        self.declare_parameter(
            'pose_topic', '/world/fleet_dynamic_capture/pose/info'
        )
        self.uav_ids = [
            str(value) for value in self.get_parameter('uav_ids').value
        ]
        self.world_name = str(self.get_parameter('world_name').value)
        self.visual_scale = max(
            0.5, float(self.get_parameter('uav_visual_scale').value)
        )
        self.pose_topic = str(self.get_parameter('pose_topic').value)

        model_share = get_package_share_directory('uav_usv_gazebo')
        model_path = (
            model_share + '/models/large_uav_visual_shell/model.sdf'
        )
        self.base_tree = ET.parse(model_path)
        self.gz_node = GzNode()
        self.visibility_node = GzNode()
        self.gz_marker_pub = self.gz_node.advertise('/marker', GzMarker)
        self.pose_lock = threading.Lock()
        self.source_poses = {}
        self.states = {}
        self.roles = {}
        self.spawned = set()
        self.hidden_sources = set()
        self.visibility_passes = {vehicle_id: 0 for vehicle_id in self.uav_ids}

        if not self.gz_node.subscribe(Pose_V, self.pose_topic, self._on_pose):
            raise RuntimeError('failed to subscribe to ' + self.pose_topic)
        self.create_subscription(
            VehicleState,
            '/fleet/state',
            self._on_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CaptureAssignmentArray,
            '/capture/roles',
            self._on_roles,
            10,
        )
        self.spawn_timer = self.create_timer(0.5, self._spawn_missing)
        self.create_timer(0.2, self._publish_labels)
        self.visibility_stop = threading.Event()
        self.visibility_thread = threading.Thread(
            target=self._visibility_worker,
            name='uav_visual_visibility',
            daemon=True,
        )
        self.visibility_thread.start()
        self.get_logger().info(
            'Visual-only UAV shells enabled: scale=%.2f, targets=%s'
            % (self.visual_scale, ','.join(self.uav_ids))
        )

    def _on_pose(self, msg):
        updates = {}
        for pose in msg.pose:
            if pose.name not in self.uav_ids:
                continue
            copied = GzPose()
            copied.CopyFrom(pose)
            updates[pose.name] = copied
        if updates:
            with self.pose_lock:
                self.source_poses.update(updates)

    def _on_state(self, msg):
        if msg.vehicle_type == VehicleState.TYPE_UAV and msg.vehicle_id:
            self.states[msg.vehicle_id] = msg

    def _on_roles(self, msg):
        self.roles = {
            assignment.vehicle_id: assignment.role_name
            for assignment in msg.assignments
        }

    @staticmethod
    def _scale_values(text, scale, count):
        values = [float(value) for value in text.split()]
        for index in range(min(count, len(values))):
            values[index] *= scale
        return ' '.join('%.6g' % value for value in values)

    @staticmethod
    def _set_material(visual, color):
        material = visual.find('material')
        if material is None:
            material = ET.SubElement(visual, 'material')
        text = ' '.join('%.3f' % value for value in color)
        for tag in ('ambient', 'diffuse'):
            element = material.find(tag)
            if element is None:
                element = ET.SubElement(material, tag)
            element.text = text

    def _append_digit(self, link, digit, center_x, color):
        for segment in self.SEGMENTS[digit]:
            x, y, yaw, size_x, size_y = self.SEGMENT_POSES[segment]
            visual = ET.SubElement(
                link, 'visual', {'name': 'id_%d_%s' % (digit, segment)}
            )
            pose = ET.SubElement(visual, 'pose')
            pose.text = '%.4f %.4f 0.315 0 0 %.4f' % (
                center_x + x, y, yaw
            )
            geometry = ET.SubElement(visual, 'geometry')
            box = ET.SubElement(geometry, 'box')
            size = ET.SubElement(box, 'size')
            size.text = '%.4f %.4f 0.025' % (size_x, size_y)
            self._set_material(visual, color)

    def _shell_sdf(self, vehicle_id, index):
        root = ET.fromstring(ET.tostring(self.base_tree.getroot()))
        model = root.find('model')
        model.set('name', vehicle_id + '_large_visual')
        plugin = model.find('plugin')
        plugin.find('target_entity').text = vehicle_id

        color = self.COLORS[index % len(self.COLORS)]
        link = model.find('link')
        accent = link.find("visual[@name='nose_accent']")
        self._set_material(accent, color)
        number = index + 1
        self._append_digit(link, 0, -0.18, (0.96, 0.96, 0.96, 1.0))
        self._append_digit(
            link, number % 10, 0.18, (0.96, 0.96, 0.96, 1.0)
        )

        for pose in model.findall('.//visual/pose'):
            pose.text = self._scale_values(
                pose.text, self.visual_scale, 3
            )
        for size in model.findall('.//box/size'):
            size.text = self._scale_values(
                size.text, self.visual_scale, 3
            )
        for mesh_scale in model.findall('.//mesh/scale'):
            mesh_scale.text = self._scale_values(
                mesh_scale.text, self.visual_scale, 3
            )
        for cylinder in model.findall('.//cylinder'):
            cylinder.find('radius').text = self._scale_values(
                cylinder.find('radius').text, self.visual_scale, 1
            )
            cylinder.find('length').text = self._scale_values(
                cylinder.find('length').text, self.visual_scale, 1
            )
        return ET.tostring(root, encoding='unicode')

    def _spawn_missing(self):
        with self.pose_lock:
            poses = dict(self.source_poses)
        for index, vehicle_id in enumerate(self.uav_ids):
            if vehicle_id in self.spawned or vehicle_id not in poses:
                continue
            request = EntityFactory()
            request.name = vehicle_id + '_large_visual'
            request.allow_renaming = False
            request.sdf = self._shell_sdf(vehicle_id, index)
            request.pose.CopyFrom(poses[vehicle_id])
            executed, response = self.gz_node.request(
                '/world/%s/create' % self.world_name,
                request,
                EntityFactory,
                Boolean,
                1000,
            )
            if executed and response.data:
                self.spawned.add(vehicle_id)
                self.get_logger().info(
                    'Spawned %s visual shell (physics entity unchanged)'
                    % vehicle_id
                )
        if len(self.spawned) == len(self.uav_ids):
            self.spawn_timer.cancel()

    def _visibility_worker(self):
        while (
            not self.visibility_stop.is_set()
            and len(self.hidden_sources) < len(self.uav_ids)
        ):
            self._hide_physical_visuals()
            time.sleep(0.5)

    def _hide_physical_visuals(self):
        pending = set(self.uav_ids) - self.hidden_sources
        if not pending:
            return
        executed, scene = self.visibility_node.request(
            '/world/%s/scene/info' % self.world_name,
            Empty(),
            Empty,
            Scene,
            1000,
        )
        if not executed:
            return
        models = {model.name: model for model in scene.model}
        for vehicle_id in pending:
            model = models.get(vehicle_id)
            shell = models.get(vehicle_id + '_large_visual')
            if model is None or shell is None:
                continue
            self.spawned.add(vehicle_id)
            # x500 geometry lives on link visuals (base plus four rotor
            # links). Toggling only the model entity leaves those visuals
            # rendered by some Gazebo GUI versions, so address every visual.
            self._set_model_visibility(model, False)
            hidden = self._set_visual_children(model, False)
            self._set_model_visibility(shell, True)
            shell_visible = self._set_visual_children(shell, True)
            if hidden and shell_visible:
                self.visibility_passes[vehicle_id] += 1
                if self.visibility_passes[vehicle_id] >= 3:
                    self.hidden_sources.add(vehicle_id)
                    self.get_logger().info(
                        'Showing only %s enlarged shell; source visuals hidden '
                        'and PX4 physics remains active' % vehicle_id
                    )
            else:
                self.visibility_passes[vehicle_id] = 0

    def _set_model_visibility(self, model, visible):
        request = Visual()
        request.id = model.id
        request.name = model.name
        request.type = Visual.MODEL
        request.visible = visible
        called, response = self.visibility_node.request(
            '/world/%s/visual_config' % self.world_name,
            request,
            Visual,
            Boolean,
            1000,
        )
        return called and response.data

    def _set_visual_children(self, model, visible):
        visual_count = 0
        for link in model.link:
            for visual in link.visual:
                visual_count += 1
                request = Visual()
                request.id = visual.id
                request.name = visual.name
                request.parent_id = link.id
                request.type = Visual.VISUAL
                request.visible = visible
                request.transparency = 0.0 if visible else 1.0
                request.cast_shadows = visible
                called, response = self.visibility_node.request(
                    '/world/%s/visual_config' % self.world_name,
                    request,
                    Visual,
                    Boolean,
                    500,
                )
                if not called or not response.data:
                    return False
        return visual_count > 0

    def destroy_node(self):
        self.visibility_stop.set()
        super().destroy_node()

    @staticmethod
    def _display_role(role):
        lowered = role.lower()
        if 'observer' in lowered:
            return 'Observer'
        if 'interceptor' in lowered:
            return 'Interceptor'
        return 'Standby'

    @staticmethod
    def _display_id(vehicle_id):
        return vehicle_id.replace('_', '-').upper()

    def _publish_labels(self):
        for index, vehicle_id in enumerate(self.uav_ids):
            state = self.states.get(vehicle_id)
            if state is None:
                continue
            marker = GzMarker()
            marker.ns = 'uav_visual_labels'
            marker.id = index + 1
            marker.action = GzMarker.ADD_MODIFY
            marker.type = GzMarker.TEXT
            marker.visibility = GzMarker.ALL
            marker.pose.position.x = float(state.pose.position.x)
            marker.pose.position.y = float(state.pose.position.y)
            marker.pose.position.z = float(
                state.pose.position.z + max(1.0, 0.55 * self.visual_scale)
            )
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.22 * self.visual_scale
            marker.scale.y = 0.22 * self.visual_scale
            marker.scale.z = 0.22 * self.visual_scale
            color = self.COLORS[index % len(self.COLORS)]
            for material in (
                marker.material.diffuse, marker.material.ambient
            ):
                material.r = color[0]
                material.g = color[1]
                material.b = color[2]
                material.a = 1.0
            marker.text = '%s  %s' % (
                self._display_id(vehicle_id),
                self._display_role(self.roles.get(vehicle_id, '')),
            )
            marker.lifetime.sec = 1
            self.gz_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = UavVisualShellSpawner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
