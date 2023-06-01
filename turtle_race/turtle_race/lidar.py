#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile

from math import degrees, cos, sin, pi
import numpy as np

from sensor_msgs.msg import Imu, LaserScan, PointCloud
from geometry_msgs.msg import Twist, Quaternion, Point32
from std_msgs.msg import Float32, String

from tf_transformations import euler_from_quaternion

import time

class obs_pub(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(PointCloud, '/obs', 1)     

    def pub(self, msg):
        msg.header.frame_id='oobbss'
        self.publisher_.publish(msg)

class IMU_LIDAR(Node):
    def __init__(self):
        super().__init__('sub')
        qos = QoSProfile(depth=10)

        self.yaw = 0.0
        self.scan = np.zeros(360)
        self.obs_xy = np.empty((1, 3))
        self.Points = PointCloud()
        
        self.pub = obs_pub()
        self.subscription = self.create_subscription(Imu,'/imu',self.listener_callback_imu,qos_profile=qos_profile_sensor_data)
        self.subscription = self.create_subscription(LaserScan,'/scan', self.listener_callback_scan, qos_profile=qos_profile_sensor_data)

    def listener_callback_imu(self, msg):
        quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]
        if self.yaw > pi:
            self.yaw %= (pi)
        elif self.yaw < -pi:
            self.yaw %= (-pi)
            
        print(degrees(self.yaw))

    def listener_callback_scan(self, msg):
        # s = time.time()
        left = -90+int(degrees(self.yaw))
        right = 90+int(degrees(self.yaw))
        # print('range,', msg.ranges[left:-1]+msg.ranges[0:right])

        self.scan = msg.ranges[left:-1]+msg.ranges[0:right]
        # print(self.scan)
        
        self.tf_tm(self.scan, self.yaw)
        # print(time.time()-s)
        
    def tf_tm(self, scan, heading, x=1.5, y=0.0):
        self.Points = PointCloud()
        self.obs_xy = np.empty((1, 3))
        resolution = 1
        T = [[cos(heading), -sin(heading), x], \
             [sin(heading),  cos(heading), y], \
             [      0     ,      0       , 1]]       # transform matrix, which is tranform lidar_xy coordinate to tm coordinate.
        
        last_scan_data = scan
        scan_data_for_search = []
        for i in range(0, len(last_scan_data)):
            scan_data_for_search.append(last_scan_data[i])
            # print('i', i)
            if np.isinf(scan_data_for_search[i]) or scan_data_for_search[i] < 0.3:
                pass
            else:
                p = Point32()
                obs_x = scan_data_for_search[i]*sin(np.deg2rad(float(i)/resolution))
                obs_y = -scan_data_for_search[i]*cos(np.deg2rad(float(i)/resolution))
                k = np.dot(T, np.transpose([obs_x, obs_y, 1]))
                k*=10
                p.x = k[0]
                p.y = k[1]
                p.z = 0.0
                # print(k)
                self.Points.points.append(p)
                self.obs_xy = np.append(self.obs_xy, [k], axis=0)
        self.obs_xy[:,2] = 0
        self.pub.pub(self.Points)

class lidar:
    def __init__(self):
        self.obs_xy = np.empty((1, 3))
        self.obs_xy_yd = np.empty((1, 3))

    def tf_tm(self, scan, heading, x=0.0, y=0.0):
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