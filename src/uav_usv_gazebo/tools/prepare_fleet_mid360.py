#!/usr/bin/env python3
"""Materialize a fleet world with an RGL Mid-360 on one existing USV.

The generated model keeps the source USV dynamics and control plugins intact.
Only semantic frames, visual geometry, and a custom RGL sensor are appended to
its existing link. No collision, inertia, joint, or control plugin is added.
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
    visual_scale,
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

    visual_frame = _element(model, 'frame', attributes={
        'name': 'mid360_visual_link',
        'attached_to': link_name,
    })
    _element(visual_frame, 'pose', mount_pose)

    def add_visual(name, z, geometry_type, dimensions, color):
        visual = _element(link, 'visual', attributes={
            'name': 'mid360_visual_' + name,
        })
        pose = _element(visual, 'pose', attributes={
            'relative_to': 'mid360_visual_link',
        })
        pose.text = '0 0 %.6f 0 0 0' % (z * visual_scale)
        geometry = _element(visual, 'geometry')
        shape = _element(geometry, geometry_type)
        if geometry_type == 'box':
            _element(
                shape,
                'size',
                ' '.join('%.6f' % (value * visual_scale)
                         for value in dimensions),
            )
        else:
            _element(shape, 'radius', '%.6f' % (
                dimensions[0] * visual_scale
            ))
            _element(shape, 'length', '%.6f' % (
                dimensions[1] * visual_scale
            ))
        material = _element(visual, 'material')
        color_text = ' '.join('%.3f' % value for value in color)
        _element(material, 'ambient', color_text)
        _element(material, 'diffuse', color_text)
        _element(material, 'specular', '0.65 0.70 0.75 1')

    # A compact CAD-style Mid-360 shell. These are visuals on the existing
    # hull link, positioned through a semantic frame, so dynamics are untouched.
    add_visual('mount', -0.025, 'box', (0.34, 0.34, 0.05),
               (0.055, 0.060, 0.065, 1.0))
    add_visual('lower_body', 0.075, 'cylinder', (0.145, 0.15),
               (0.075, 0.080, 0.085, 1.0))
    add_visual('shoulder', 0.175, 'cylinder', (0.132, 0.055),
               (0.12, 0.13, 0.14, 1.0))
    add_visual('scan_window', 0.245, 'cylinder', (0.125, 0.085),
               (0.025, 0.12, 0.16, 0.92))
    add_visual('top_cap', 0.310, 'cylinder', (0.108, 0.045),
               (0.045, 0.050, 0.055, 1.0))
    add_visual('status_bar', 0.105, 'box', (0.012, 0.30, 0.035),
               (0.15, 0.80, 0.90, 1.0))

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
        visual_scale=args.visual_scale,
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
    parser.add_argument('--visual-scale', type=float, default=1.0)
    args = parser.parse_args()

    if args.update_rate <= 0.0:
        parser.error('--update-rate must be positive')
    if args.min_range < 0.0 or args.max_range <= args.min_range:
        parser.error('invalid Mid-360 range')
    if args.visual_scale <= 0.0:
        parser.error('--visual-scale must be positive')
    prepare(args)


if __name__ == '__main__':
    main()
