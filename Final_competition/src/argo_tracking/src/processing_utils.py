#!/usr/bin/env python
import numpy as np
import re
from scipy import optimize
import math
import json
from pathlib import Path
RATE = 10

# for hamming smoothing
WINDOW_SIZE = 11

# for kalman filter
TRANSITION_MATRIX = [[1, 1, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, 1],
                     [0, 0, 0, 1]]

OBSERVATION_MATRIX = [[1, 0, 0, 0],
                      [0, 0, 1, 0]]

def hamming_smoothing(signal, window_size):
    padded = np.r_[signal[window_size-1:0:-1], signal, signal[-2:-window_size-1:-1]] # pad the signal at two ends
    window = np.hamming(window_size)
    smoothed = np.convolve(window/window.sum(), padded, mode='valid')
    return smoothed[window_size/2-1:-window_size/2]

def circle_fitting(locations, prediction_points=5):
    """
    Fit a circle on locations using least square.
    Return :    Predictions of x and y
                Tangent angle at the last prediction position
    """
    x = locations[:, 0]
    y = locations[:, 1]

    x_m = np.mean(x)
    y_m = np.mean(y)

    def calc_R(xc, yc):
        """ calculate the distance of each 2D points from the center (xc, yc) """
        return np.sqrt((x-xc)**2 + (y-yc)**2)

    def f(c):
        """ calculate the algebraic distance between the data points and the mean circle centered at c=(xc, yc) """
        Ri = calc_R(*c)
        return Ri - Ri.mean()

    center_estimate = x_m, y_m
    center, _ = optimize.leastsq(f, center_estimate)

    xc, yc = center
    Ri = calc_R(xc, yc)
    R = Ri.mean()

    angles = np.arctan2(y-yc, x-xc)

    angles_diff = angles[:-1]-angles[1:]
    angles_diff[angles_diff<-np.pi] += 2*np.pi
    mean_angle_variation = np.mean(np.minimum(angles_diff, 2*np.pi-angles_diff))

    angle_prediction = np.linspace(0, (prediction_points-1)*mean_angle_variation, prediction_points) + angles[0]
    x_prediction = R*np.cos(angle_prediction)+xc
    y_prediction = R*np.sin(angle_prediction)+yc

    tangent_angle = angle_prediction[-1] + np.pi/2 # the tangent angle is 90 degrees more
    if mean_angle_variation < 0: # if its movement is clock-wise, flip the direction of 180 degrees
        tangent_angle += np.pi

    return np.array(zip(x_prediction, y_prediction)), tangent_angle

def natural_key(string_):
    return [int(s) if s.isdigit() else s for s in re.split(r'(\d+)', string_)]

def distance(pose, prev_pose):
    return np.linalg.norm(pose - prev_pose)
def euler_from_quaternion(w,x, y, z):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
 
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
 
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
 
    return roll_x, pitch_y, yaw_z # in radians


def string_split(data_path_list):
    """
    Get the timestemp of the list which contains file names

    Return :
    list contain only timestemp
    """
    time_stamp = []
    for i in range(len(data_path_list)):
        p = (Path(data_path_list[i]).stem).split('_')
        #print(p[-1])
        time_stamp += [p[-1]]
    return time_stamp
def readJson(pose_files):
    with open(pose_files) as f:
        pose_data = json.load(f)
        pose = np.array(pose_data['translation'])
        rotation = np.array(pose_data['rotation'])
        r,p,y = euler_from_quaternion(rotation[0],rotation[1],rotation[2],rotation[3]) 
        # returns Json object as a dictionary
        
    return pose, y