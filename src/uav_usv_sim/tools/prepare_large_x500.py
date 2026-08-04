#!/usr/bin/env python3
"""Generate a physically scaled PX4 x500 model from an immutable backup."""

import argparse
import os
import shutil
import xml.etree.ElementTree as ET


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--px4-dir', required=True)
    parser.add_argument('--scale', type=float, default=3.5)
    parser.add_argument('--camera-width', type=int, default=640)
    parser.add_argument('--camera-height', type=int, default=480)
    parser.add_argument('--camera-rate', type=float, default=20.0)
    return parser.parse_args()


def scale_values(text, factors):
    values = [float(value) for value in text.split()]
    for index, factor in enumerate(factors):
        if index < len(values):
            values[index] *= factor
    return ' '.join(f'{value:.10g}' for value in values)


def scale_model(source, destination, scale):
    tree = ET.parse(source)
    root = tree.getroot()

    for pose in root.iter('pose'):
        if pose.text:
            pose.text = scale_values(pose.text, (scale, scale, scale))

    for geometry in root.iter('geometry'):
        for size in geometry.iter('size'):
            if size.text:
                size.text = scale_values(size.text, (scale, scale, scale))
        for mesh_scale in geometry.iter('scale'):
            if mesh_scale.text:
                mesh_scale.text = scale_values(
                    mesh_scale.text,
                    (scale, scale, scale),
                )
        for tag in ('radius', 'length'):
            for value in geometry.iter(tag):
                if value.text:
                    value.text = f'{float(value.text) * scale:.10g}'

    mass_scale = scale ** 3
    inertia_scale = scale ** 5
    for inertial in root.iter('inertial'):
        mass = inertial.find('mass')
        if mass is not None and mass.text:
            mass.text = f'{float(mass.text) * mass_scale:.10g}'
        inertia = inertial.find('inertia')
        if inertia is None:
            continue
        for tag in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'):
            value = inertia.find(tag)
            if value is not None and value.text:
                value.text = f'{float(value.text) * inertia_scale:.10g}'

    root.insert(
        0,
        ET.Comment(
            f' UAV_USV generated large x500, geometric scale={scale:g} '
        ),
    )
    tree.write(destination, encoding='UTF-8', xml_declaration=True)


def scale_motor_model(source, destination, scale):
    """Scale rotor coefficients for a geometrically similar airframe.

    The PX4 SITL motor interface uses a normalized collective command with a
    bounded hover-thrust range.  Scaling the force coefficient with ``s^3``
    preserves the stock x500 hover command after the airframe mass is scaled
    by ``s^3``.  The increased rotational inertia is compensated by the PX4
    rate-controller profile loaded by the fleet launch, rather than pushing
    PX4 below its supported hover-thrust range.
    """
    tree = ET.parse(source)
    root = tree.getroot()
    force_scale = scale ** 3
    torque_scale = scale ** 4
    for plugin in root.findall(".//plugin"):
        if 'MulticopterMotorModel' not in plugin.get('name', ''):
            continue
        for tag, factor in (
            ('motorConstant', force_scale),
            ('momentConstant', scale),
            ('rotorDragCoefficient', force_scale),
            ('rollingMomentCoefficient', torque_scale),
        ):
            value = plugin.find(tag)
            if value is not None and value.text:
                value.text = f'{float(value.text) * factor:.10g}'
    root.insert(
        0,
        ET.Comment(f' UAV_USV generated x500 motor scale={scale:g} '),
    )
    tree.write(destination, encoding='UTF-8', xml_declaration=True)


def prepare_file(path, scale):
    backup = path + '.uav_usv_unscaled'
    if not os.path.exists(backup):
        shutil.copy2(path, backup)
    scale_model(backup, path, scale)


def prepare_camera(path, width, height, rate, scale):
    backup = path + '.uav_usv_unscaled'
    if not os.path.exists(backup):
        shutil.copy2(path, backup)

    scale_model(backup, path, scale)
    tree = ET.parse(path)
    root = tree.getroot()
    sensor = root.find(".//sensor[@type='camera']")
    if sensor is None:
        raise RuntimeError(f'No camera sensor found in {path}')
    image = sensor.find('./camera/image')
    if image is None:
        raise RuntimeError(f'No camera image configuration found in {path}')
    image.find('width').text = str(width)
    image.find('height').text = str(height)
    sensor.find('update_rate').text = f'{rate:g}'
    # Keep the optical origin in front of the enlarged lens.  Without this
    # offset the down-facing camera can render its own lens housing.
    sensor_pose = sensor.find('pose')
    if sensor_pose is not None:
        sensor_pose.text = f'{0.025 * scale:.10g} 0 0 0 0 0'
    visualize = sensor.find('visualize')
    if visualize is not None:
        visualize.text = 'false'
    root.insert(
        0,
        ET.Comment(
            f' UAV_USV camera {width}x{height} at {rate:g} FPS '
        ),
    )
    tree.write(path, encoding='UTF-8', xml_declaration=True)


def main():
    args = parse_args()
    model_root = os.path.join(
        os.path.expanduser(args.px4_dir),
        'Tools',
        'simulation',
        'gz',
        'models',
    )
    physical_paths = (
        os.path.join(model_root, 'x500_base', 'model.sdf'),
        os.path.join(model_root, 'x500_mono_cam_down', 'model.sdf'),
    )
    for path in physical_paths:
        if not os.path.isfile(path):
            raise FileNotFoundError(path)
        prepare_file(path, args.scale)
    motor_path = os.path.join(model_root, 'x500', 'model.sdf')
    if not os.path.isfile(motor_path):
        raise FileNotFoundError(motor_path)
    motor_backup = motor_path + '.uav_usv_unscaled'
    if not os.path.exists(motor_backup):
        shutil.copy2(motor_path, motor_backup)
    scale_motor_model(motor_backup, motor_path, args.scale)
    camera_path = os.path.join(model_root, 'mono_cam', 'model.sdf')
    if not os.path.isfile(camera_path):
        raise FileNotFoundError(camera_path)
    prepare_camera(
        camera_path,
        args.camera_width,
        args.camera_height,
        args.camera_rate,
        args.scale,
    )
    print(f'Prepared PX4 x500 physical scale: {args.scale:g}x')
    print(
        'Prepared PX4 camera: '
        f'{args.camera_width}x{args.camera_height}@{args.camera_rate:g}'
    )


if __name__ == '__main__':
    main()
