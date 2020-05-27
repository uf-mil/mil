#!/usr/bin/env python3
import yaml
import argparse
import numpy as np
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(
    description='Generates the points down the centerline of a reace track given specified by a yaml file')
# get input yaml specifying racetrack
parser.add_argument(
    'input_file',
    help='name of the input yaml file specifying the racetrack')
# get output file name
parser.add_argument('output_file', help='location of output xacro file')
# get debug plot or not
parser.add_argument(
    '--debug',
    action='store_true',
    help='if present, will plot the points of the track')

args = parser.parse_args()


def points_as_complex(pose, radius, center_dist, resolution):
    azimuth = pose['orientation']['azimuth']
    orientation = complex(
        np.cos(
            (90 - azimuth) * np.pi / 180),
        np.sin(
            (90 - azimuth) * np.pi / 180)) * 1j
    center_of_rotation = complex(
        pose['orientation']['center_of_rotation']['x'],
        pose['orientation']['center_of_rotation']['y'])
    translation = complex(pose['translation']['x'], pose['translation']['y'])

    points = np.zeros(
        (int(
            2 *
            np.pi *
            radius *
            resolution),
         ),
        dtype=np.complex)
    steps = np.linspace(0, 2 * np.pi, points.size)
    points = np.cos(steps) + np.sin(steps) * 1j
    points *= radius
    points[:points.size // 2] += center_dist * 1j / 2
    points[points.size // 2:] -= center_dist * 1j / 2
    points = np.hstack((points, points[0],))
    points += center_of_rotation  # set the center of rotation
    points *= orientation  # rotate all the points
    points += translation  # translate all the points to be at the right spot in gazebo
    if (args.debug):
        plt.plot(points.real, points.imag)
        plt.show()
    return points


def points_as_str(*args):
    points = points_as_complex(*args)
    points = np.array([points.real, points.imag]).transpose()
    points = np.hstack((points, np.zeros((points.shape[0], 1))))
    ret = ''
    for i in range(points.shape[0]):
        ret += '    <point> ' + str(points[i, 0]) + \
            ' ' + str(points[i, 1]) + \
            ' ' + str(points[i, 2]) + '</point>\n'
    return ret


if __name__ == '__main__':
    with open(args.input_file) as input_file:
        track = yaml.safe_load(input_file)
    out = '<?xml version=\'1.0\' encoding=\'ASCII\'?>\n'
    out += '<world xmlns:xacro="http://ros.org/wiki/xacro">\n'
    out += '  <xacro:macro name="' + track['track_name'] + '">\n'
    out += '    <road name="' + track['track_name'] + '">\n'
    out += '      <width>' + str(track['width']) + '</width>\n'
    out += points_as_str(track['pose'],
                         track['radius'],
                         track['center_dist'],
                         track['resolution'])
    out += '    </road>\n'
    out += '  </xacro:macro>'
    out += '</world>\n'
    output_file = open(args.output_file, "w+")
    output_file.write(out)
    output_file.close()
