#!/usr/bin/env python
import rclpy
from rclpy.node import Node

from math import sin, cos, pi
import numpy as np
from scipy.spatial import distance

from geometry_msgs.msg import Twist

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
    
    def get_steer(self,obs):
        self.get_path(obs)
        start = self.path[-1]
        goal = self.path[-LD]
        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        
        return dx/dy*W

def main(args=None):
    rclpy.init(args=args)
    sensor = lidar.IMU_LIDAR()
    motor = Motor_Pub()
    Race = race()
    vel = Twist()
    
    while rclpy.ok():
        rclpy.spin_once(sensor)
        steer = Race.get_steer(sensor.obs_xy)
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