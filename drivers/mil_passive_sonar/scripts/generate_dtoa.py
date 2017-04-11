#!/usr/bin/env python
from multilateration import *
import numpy as np
import argparse

h0 = {'x' : 0,     'y': 0,    'z' : 0}
h1 = {'x' : 25.4,  'y': 0,    'z' : 0}
h2 = {'x' : -25.4, 'y': 0,    'z' : 0}
h3 = {'x' : 0,     'y': 25.4, 'z' : 0}
hydrophone_locations = [h1, h2, h3]

def dict_to_xyz_vec(pos_dict):
    '''
    Converts a dictionary with 'x', 'y', and 'z' keys to a numpy array
    '''
    p = pos_dict
    return np.array([p['x'], p['y'], p['z']])

def get_dtoa(c, sensor_position, source):
    '''
    Calculates the dtoa for a signal and a sensor that is ref_offset away from the reference sensor
    c should be in millimeter per microsecond
    position params should have units of mm
    '''
    assert(isinstance(sensor_position, np.ndarray))
    assert(isinstance(source, np.ndarray))
    assert(len(sensor_position) == 3)
    assert(len(source) == 3)
    t_ref = np.linalg.norm(source) / c
    t = np.linalg.norm(source - sensor_position) / c
    delta_t = t_ref - t
    #print __name__, sensor_position, source, c, t_ref, t, delta_t
    return delta_t

def get_batch_dtoa(c, sensor_positions, source):
    '''
    Calculates the dtoa's of a signal to a list of sensors(positions). The reference
    is assumed to be at the origin.
    c should be in millimeter per microsecond
    position params should have units of mm
    '''
    dtoa = []
    for x in sensor_positions:
        dtoa.append(get_dtoa(c, x, source))
    return dtoa

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Generates dtoa measurements given sensor and source locations',
        usage='Pass the path to a file with source locations. Each row should have 3 columns' +
              'The columns should have the x, y, and z coordinates of each source location, ' \
              'separated by spaces or tabs. Coordinates should be in units of millimeters.')
    parser.add_argument('path_to_input')
    args = parser.parse_args()
    sonar_test = np.loadtxt(args.path_to_input)
    sensors = [dict_to_xyz_vec(x) for x in hydrophone_locations]
    res = []
    for case in sonar_test:
        res.append(get_batch_dtoa(1.484, sensors, case))
    res = np.array(res)
    # output format is {numbers of input rows} rows w/ 3 columns (dtoa measurements in microseconds)
    print res 

