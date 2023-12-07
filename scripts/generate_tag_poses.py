#!/usr/bin/env python3
import argparse
import os
import sys
from typing import Tuple

import numpy as np
import tf_transformations
import yaml


def float_representer(dumper, value):
    text = '{0:.3f}'.format(value)
    return dumper.represent_scalar(u'tag:yaml.org,2002:float', text)


def generate_even_grid(size: Tuple[int, int], offset: Tuple[float, float,
                                                            float],
                       distance_between_tags: Tuple[float, float], tag_size):
    data = {}
    data['tag_poses'] = []
    tag_id = 0
    for row in range(size[0]):
        for col in range(size[1]):
            x = col * distance_between_tags[0] + offset[0]
            y = row * distance_between_tags[1] + offset[1]
            z = offset[2]
            quat = tf_transformations.quaternion_from_euler(0, 0, 0)
            data['tag_poses'].append({
                'frame_id': 'map',
                'id': tag_id,
                'size': tag_size,
                'x': x,
                'y': y,
                'z': z,
                'qw': float(quat[3]),
                'qx': float(quat[0]),
                'qy': float(quat[1]),
                'qz': float(quat[2]),
            })
            tag_id += 1
    return data


def generate_standalone_tags(n_tags: int, size: float):
    data = {}
    data['standalone_tags'] = {}
    data['standalone_tags']['tag_names'] = []
    for i in range(n_tags):
        tag_name = f'tag_{i}'
        data['standalone_tags']['tag_names'].append(tag_name)
        data['standalone_tags'][tag_name] = {'id': i, 'size': size}
    return data


def generate_tag_bundle(tag_poses, name: str):
    layout = {'ids': []}
    for tag in tag_poses['tag_poses']:
        layout['ids'].append(tag['id'])
        layout[tag["id"]] = {
            'size': tag['size'],
            'x': tag['x'],
            'y': tag['y'],
            'z': tag['z'],
            'qw': 1.0,
            'qx': 0.0,
            'qy': 0.0,
            'qz': 0.0,
        }
    data = {f'{name}': {'layout': layout}}
    return data


def convert_to_rosparam(data):
    return {'/**': {'ros__parameters': data}}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--tag-size',
                        default=0.05,
                        help='Tag size in meters including the border.')
    parser.add_argument('--grid-size',
                        nargs=2,
                        default=[13, 7],
                        type=int,
                        help='Number of rows and cols of the tag grid.')
    parser.add_argument('--offset',
                        nargs=3,
                        default=[0.2, 0.3, -1.45],
                        type=float,
                        help='Offset of the tag grid relative to the origin')
    parser.add_argument('--distance',
                        nargs=2,
                        default=[0.25, 0.3],
                        type=float,
                        help='Distance between tags in x- and y-direction')
    parser.add_argument('--out-dir',
                        required=True,
                        help='Output directory of the generated files.')
    args = parser.parse_args()

    yaml.add_representer(float, float_representer)
    pose_data = generate_even_grid(args.grid_size, args.offset, args.distance,
                                   args.tag_size)
    if args.out_dir == '-':
        yaml.dump(pose_data,
                  sys.stdout,
                  default_flow_style=True,
                  width=float('inf'))
    else:
        filename = 'tag_poses.yaml'
        filepath = os.path.join(args.out_dir, filename)
        with open(filepath, 'w') as f:
            yaml.dump(pose_data, f)
            print(f'Created file [{filepath}]')

        # filename = 'tags_standalone.yaml'
        # filepath = os.path.join(args.out_dir, filename)
        # n = args.grid_size[0] * args.grid_size[1]
        # # correct for difference in physical tag size and tag without border
        # standalone_data = generate_standalone_tags(n, args.tag_size * 8 / 10)
        # with open(filepath, 'w') as f:
        #     yaml.dump(convert_to_rosparam(standalone_data), f)
        #     print(f'Created file [{filepath}]')

        # filename = 'tag_bundle.yaml'
        # filepath = os.path.join(args.out_dir, filename)
        # bundle_data = generate_tag_bundle(pose_data, 'all_tags')
        # with open(filepath, 'w') as f:
        #     yaml.dump(convert_to_rosparam(bundle_data), f)
        #     print(f'Created file [{filepath}]')

        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)
            num_tags = len(data['tag_poses'])

            # iterate over tags:
            tags = np.zeros((num_tags, 8))
            for tag in data['tag_poses']:
                if tag['frame_id'] == 'map':
                    tags[tag['id'], :] = np.array([
                        tag['id'], tag['x'], tag['y'], tag['z'], tag['qx'],
                        tag['qy'], tag['qz'], tag['qw']
                    ])
                else:
                    print('Tag not in map frame! - not implemented yet')


if __name__ == '__main__':
    main()
