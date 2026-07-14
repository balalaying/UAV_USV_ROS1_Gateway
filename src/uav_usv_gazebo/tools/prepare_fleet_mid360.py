#!/usr/bin/env python3
"""Materialize a fleet world with an RGL Mid-360 on one existing USV.

The generated model keeps the source USV dynamics and control plugins intact.
Only an SDF frame, a visual, and a custom RGL sensor are appended to its
existing link.
"""

import argparse
import os
import shutil
import xml.etree.ElementTree as ET


def _element(parent, tag, text=None, attributes=None):
    child = ET.SubElement(parent, tag, attributes or {})
    if text is not None:
        child.text = str(text)
    return child


def _find_vehicle_model(world, vehicle_id):
    for include in world.findall('include'):
        name = include.findtext('name', '').strip()
        if name != vehicle_id:
            continue
        uri = include.findtext('uri', '').strip()
        if not uri.startswith('model://'):
            raise RuntimeError(
                'Vehicle %s does not use a model:// URI: %s'
                % (vehicle_id, uri)
            )
        return include, uri[len('model://'):]
    raise RuntimeError('Vehicle %s was not found in the world' % vehicle_id)


def _append_manager(world):
    for plugin in world.findall('plugin'):
        if plugin.get('filename') == 'RGLServerPluginManager':
            return
    manager = ET.Element('plugin', {
        'filename': 'RGLServerPluginManager',
        'name': 'rgl::RGLServerPluginManager',
    })
    _element(manager, 'do_ignore_entities_in_lidar_link', 'true')
    world.insert(0, manager)


def _append_mid360(
    model,
    link_name,
    mount_pose,
    raw_topic,
    frame_id,
    update_rate,
    min_range,
    max_range,
):
    link = model.find("link[@name='%s']" % link_name)
    if link is None:
        raise RuntimeError(
            'Link %s was not found in model %s'
            % (link_name, model.get('name', '<unnamed>'))
        )
    if model.find("frame[@name='mid360_link']") is not None:
        raise RuntimeError('Source model already contains mid360_link')

    frame = _element(model, 'frame', attributes={
        'name': 'mid360_link',
        'attached_to': link_name,
    })
    _element(frame, 'pose', mount_pose)

    visual = _element(link, 'visual', attributes={'name': 'mid360_visual'})
    _element(visual, 'pose', mount_pose)
    geometry = _element(visual, 'geometry')
    cylinder = _element(geometry, 'cylinder')
    _element(cylinder, 'radius', '0.16')
    _element(cylinder, 'length', '0.16')
    material = _element(visual, 'material')
    _element(material, 'ambient', '0.03 0.25 0.28 1')
    _element(material, 'diffuse', '0.05 0.68 0.72 1')
    _element(material, 'specular', '0.8 0.9 0.95 1')

    sensor = _element(link, 'sensor', attributes={
        'name': 'mid360_rgl',
        'type': 'custom',
    })
    _element(sensor, 'pose', mount_pose)
    plugin = _element(sensor, 'plugin', attributes={
        'filename': 'RGLServerPluginInstance',
        'name': 'rgl::RGLServerPluginInstance',
    })
    sensor_range = _element(plugin, 'range')
    _element(sensor_range, 'min', min_range)
    _element(sensor_range, 'max', max_range)
    _element(plugin, 'update_rate', update_rate)
    _element(plugin, 'update_on_paused_sim', 'false')
    _element(plugin, 'topic', raw_topic)
    _element(plugin, 'frame', frame_id)
    _element(plugin, 'pattern_preset', 'Livox Mid360')


def prepare(args):
    world_tree = ET.parse(args.world)
    world_root = world_tree.getroot()
    world = world_root.find('world')
    if world is None:
        raise RuntimeError('No <world> element in %s' % args.world)

    vehicle_include, model_name = _find_vehicle_model(
        world, args.vehicle_id
    )
    source_model_dir = os.path.join(args.models_dir, model_name)
    source_model = os.path.join(source_model_dir, 'model.sdf')
    if not os.path.isfile(source_model):
        raise RuntimeError('USV model SDF not found: %s' % source_model)

    model_tree = ET.parse(source_model)
    model = model_tree.getroot().find('model')
    if model is None:
        raise RuntimeError('No <model> element in %s' % source_model)

    _append_manager(world)
    _append_mid360(
        model=model,
        link_name=args.link_name,
        mount_pose=args.mount_pose,
        raw_topic=args.raw_topic,
        frame_id=args.frame_id,
        update_rate=args.update_rate,
        min_range=args.min_range,
        max_range=args.max_range,
    )

    runtime_model_name = model_name + '_mid360_runtime'
    output_model_dir = os.path.join(
        args.output_root, 'models', runtime_model_name
    )
    output_world_dir = os.path.join(args.output_root, 'worlds')
    os.makedirs(output_model_dir, exist_ok=True)
    os.makedirs(output_world_dir, exist_ok=True)

    config = os.path.join(source_model_dir, 'model.config')
    if os.path.isfile(config):
        shutil.copy2(config, os.path.join(output_model_dir, 'model.config'))
    model_tree.write(
        os.path.join(output_model_dir, 'model.sdf'),
        encoding='utf-8',
        xml_declaration=True,
    )
    vehicle_include.find('uri').text = 'model://' + runtime_model_name
    output_world = os.path.join(
        output_world_dir, os.path.basename(args.world)
    )
    world_tree.write(output_world, encoding='utf-8', xml_declaration=True)
    print(output_world)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', required=True)
    parser.add_argument('--models-dir', required=True)
    parser.add_argument('--output-root', required=True)
    parser.add_argument('--vehicle-id', default='usv_01')
    parser.add_argument('--link-name', default='hull')
    parser.add_argument('--mount-pose', default='0.9075 0 1.5625 0 0 0')
    parser.add_argument('--raw-topic', required=True)
    parser.add_argument('--frame-id', required=True)
    parser.add_argument('--update-rate', type=float, default=10.0)
    parser.add_argument('--min-range', type=float, default=0.1)
    parser.add_argument('--max-range', type=float, default=70.0)
    args = parser.parse_args()

    if args.update_rate <= 0.0:
        parser.error('--update-rate must be positive')
    if args.min_range < 0.0 or args.max_range <= args.min_range:
        parser.error('invalid Mid-360 range')
    prepare(args)


if __name__ == '__main__':
    main()
