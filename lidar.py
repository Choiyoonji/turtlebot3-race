#!/usr/bin/env python
import rospy

from math import cos, sin, pi
import numpy as np

class lidar:
    def __init__(self):
        self.obs_xy = np.empty((1, 3))
        self.obs_xy_yd = np.empty((1, 3))

    def tf_tm(self, scan, x, y, heading) :
        self.obs_xy = np.empty((1, 3))
        resolution = 1
        T = [[cos(heading), -sin(heading), x], \
             [sin(heading),  cos(heading), y], \
             [      0     ,      0       , 1]]       # transform matrix, which is tranform lidar_xy coordinate to tm coordinate.
        
        last_scan_data = scan
        scan_data_for_search = []
        for i in range(0, 360):
            scan_data_for_search.append(last_scan_data[i])
            if np.isinf(scan_data_for_search[i]) or scan_data_for_search[i] < 0.1:
                pass
            elif scan_data_for_search[i] <= 15:
                obs_x = scan_data_for_search[i]*sin(np.deg2rad(float(i)/resolution))
                obs_y = -scan_data_for_search[i]*cos(np.deg2rad(float(i)/resolution))
                self.obs_xy = np.append(self.obs_xy, [np.dot(T, np.transpose([obs_x, obs_y, 1]))], axis=0)
        self.obs_xy[:,2] = 0
        
        return self.obs_xy