#!/usr/bin/env python
import rclpy
from rclpy.node import Node

from math import sin, cos, pi, asin
import numpy as np
from scipy.spatial import distance

from geometry_msgs.msg import Twist

import sys
sys.path.append(r'/home/choiyoonji/ros2_ws/src/turtlebot3-race/turtle_race/turtle_race/')

import astar
import lidar

LD = 7
W = 1
SPEED = 0.5

class Motor_Pub(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)     

    def pub(self, msg):
        self.publisher_.publish(msg)

class race():
    def __init__(self):
        self.path = []
    
    def get_path(self, obs):
        Astar = astar.Astar()
        self.path = Astar.generate_path(obs)
    
    def get_steer(self, obs, heading):
        self.get_path(obs)
        
        start = self.path[-1]
        goal = self.path[-LD]
        
        dl = distance.euclidean(start,goal)
        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        
        theta = asin(dy/dl)
        
        if dx > 0:
            theta = 90 - theta
        elif dx < 0:
            theta = -(90 - theta)
        else:
            theta = 0
        
        return (heading - theta) * W

def main(args=None):
    rclpy.init(args=args)
    sensor = lidar.IMU_LIDAR()
    motor = Motor_Pub()
    Race = race()
    vel = Twist()
    
    while rclpy.ok():
        rclpy.spin_once(sensor)
        steer = Race.get_steer(sensor.obs_xy, sensor.heading)
        vel.linear.x = SPEED
        vel.angular.z = steer
        motor.pub(vel)
        
    vel.linear.x = 0.0
    vel.angular.z = 0.0
    motor.pub(vel)
    
    sensor.destroy_node()
    motor.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()