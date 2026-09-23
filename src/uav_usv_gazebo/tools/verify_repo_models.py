#!/usr/bin/env python3
"""Fail fast unless heterogeneous_332 resolves to repository-owned models."""

import argparse
import os
import sys
import xml.etree.ElementTree as ET


EXPECTED_WORLD_MODELS = {
    'uav_01': 'ardupilot_uav_01',
    'uav_02': 'ardupilot_uav_02',
    'uav_03': 'ardupilot_uav_03',
    'usv_01': 'sim332_usv_blue',
    'usv_02': 'sim332_usv_green',
    'usv_03': 'sim332_usv_cyan',
    'friendly_ship': 'sim332_friendly_ship',
    'enemy_ship': 'sim332_enemy_ship',
}

EXPECTED_USV_BASES = {
    'sim332_usv_blue': 'defense_own_01_boat',
    'sim332_usv_green': 'defense_own_02_boat',
    'sim332_usv_cyan': 'defense_own_03_boat',
}

EXPECTED_SENSOR_TOPICS = {
    'defense_own_01_boat': {
        '/defense/own_01/front_camera',
        '/defense/own_01/depth_camera',
        '/defense/own_01/scan',
    },
    'defense_own_02_boat': {
        '/defense/own_02/front_camera',
        '/defense/own_02/depth_camera',
        '/defense/own_02/scan',
    },
    'defense_own_03_boat': {
        '/defense/own_03/front_camera',
        '/defense/own_03/depth_camera',
        '/defense/own_03/scan',
    },
}


def includes(document):
    result = []
    for include in document.findall('.//include'):
        uri = include.findtext('uri', '').strip()
        name = include.findtext('name', '').strip()
        if uri.startswith('model://'):
            result.append((name, uri[len('model://'):].split('/', 1)[0]))
    return result


def parse(path):
    try:
        return ET.parse(path)
    except (ET.ParseError, OSError) as error:
        raise RuntimeError('%s: %s' % (path, error))


def require_model(model_root, model_name):
    path = os.path.realpath(os.path.join(model_root, model_name, 'model.sdf'))
    root = os.path.realpath(model_root) + os.sep
    if not path.startswith(root) or not os.path.isfile(path):
        raise RuntimeError(
            'required repository model is missing: %s' % model_name
        )
    return path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', required=True)
    parser.add_argument('--model-root', required=True)
    args = parser.parse_args()

    world_path = os.path.realpath(args.world)
    model_root = os.path.realpath(args.model_root)
    world = parse(world_path)
    world_models = {name: model for name, model in includes(world)}
    errors = []
    for entity, expected in EXPECTED_WORLD_MODELS.items():
        actual = world_models.get(entity)
        if actual != expected:
            errors.append(
                '%s must use model://%s, found %r' % (
                    entity, expected, actual
                )
            )

    for wrapper, base in EXPECTED_USV_BASES.items():
        wrapper_path = require_model(model_root, wrapper)
        wrapper_models = {name: model for name, model in includes(parse(wrapper_path))}
        if base not in wrapper_models.values():
            errors.append('%s no longer includes %s' % (wrapper, base))
        base_path = require_model(model_root, base)
        topics = {
            (element.text or '').strip()
            for element in parse(base_path).findall('.//sensor/topic')
        }
        missing_topics = EXPECTED_SENSOR_TOPICS[base] - topics
        if missing_topics:
            errors.append(
                '%s lost sensor topics: %s' % (
                    base, ', '.join(sorted(missing_topics))
                )
            )
        print('USV_MODEL %s -> %s -> %s' % (
            wrapper, base, base_path
        ))

    for entity in ('friendly_ship', 'enemy_ship'):
        model = EXPECTED_WORLD_MODELS[entity]
        print('SHIP_MODEL %s -> %s' % (
            entity, require_model(model_root, model)
        ))
    for entity in ('uav_01', 'uav_02', 'uav_03'):
        model = EXPECTED_WORLD_MODELS[entity]
        path = require_model(model_root, model)
        text = open(path, encoding='utf-8').read()
        if 'ArduPilotPlugin' not in text:
            errors.append('%s has no ArduPilotPlugin' % model)
        print('UAV_MODEL %s -> %s' % (entity, path))

    if errors:
        for error in errors:
            print('MODEL_ERROR ' + error, file=sys.stderr)
        return 2
    print('MODEL_POLICY repository models verified for ' + world_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
