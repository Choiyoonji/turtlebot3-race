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

import pygame

LD = 15
W = 1
SPEED = 0.0

class Motor_Pub(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)     

    def pub(self, msg):
        self.publisher_.publish(msg)

class race():
    def __init__(self):
        self.path = []
        # self.Astar = astar.Astar()
    
    def get_path(self, obs):
        self.Astar = astar.Astar()
        self.path = self.Astar.generate_path(obs)
    
    def get_steer(self, obs, heading):
        self.get_path(obs)
        
        start = self.path[-1]
        goal = self.path[-LD]
        # print(goal)
        
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
    # obs = lidar.obs_pub()
    motor = Motor_Pub()
    Race = race()
    vel = Twist()
    
    pygame.init() #파이 게임 초기화
    SCREEN_WIDTH = 1200
    SCREEN_HEIGHT = 600
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT)) #화면 크기 설정
    clock = pygame.time.Clock() 

    #변수

    BLACK = (0, 0, 0)
    GRAY = (128, 128, 128)
    RED = (255, 0, 0)
    BLUE = (0, 255, 0)

    CELL_SIZE = 2
    COLUMN_COUNT = SCREEN_WIDTH // CELL_SIZE
    ROW_COUNT = SCREEN_HEIGHT // CELL_SIZE

    bodies = [(COLUMN_COUNT // 2, ROW_COUNT // 2)]
    
    
    
    while rclpy.ok():
        rclpy.spin_once(sensor)
        # print('fjewofiwef')
        # print(sensor.obs_xy)
        
        # steer = Race.get_steer(sensor.obs_xy, sensor.yaw)
        # # print(sensor.obs_xy)
        # # print('yaw',sensor.yaw)
        # # print(steer)
        # vel.linear.x = SPEED
        # vel.angular.z = steer
        # motor.pub(vel)
        # # obs.pub(sensor.Points)
        # screen.fill(BLACK) #단색으로 채워 화면 지우기

        # #변수 업데이트

        # event = pygame.event.poll() #이벤트 처리

        # #화면 그리기

        # for column_index in range(COLUMN_COUNT):
        #     for row_index in range(ROW_COUNT):
        #         rect = (CELL_SIZE * column_index, CELL_SIZE * row_index, CELL_SIZE, CELL_SIZE)
        #         if Race.Astar.MAP[row_index,column_index] == 0:
        #             pygame.draw.rect(screen, BLACK, rect, 1)
        #         elif Race.Astar.MAP[row_index,column_index] == -1:
        #             pygame.draw.rect(screen, RED, rect, 1)
        #         elif Race.Astar.MAP[row_index,column_index] == 5:
        #             pygame.draw.rect(screen, BLUE, rect, 1)
        #         elif Race.Astar.MAP[row_index,column_index] == 2:
        #             pygame.draw.rect(screen, GRAY, rect, 1)

        # pygame.display.update() #모든 화면 그리기 업데이트
        # clock.tick(5) #30 FPS (초당 프레임 수) 를 위한 딜레이 추가, 딜레이 시간이 아닌 목표로 하는 FPS 값
        
    vel.linear.x = 0.0
    vel.angular.z = 0.0
    motor.pub(vel)
    
    sensor.destroy_node()
    motor.destroy_node()
    rclpy.shutdown()
    
    pygame.quit() 
    
if __name__ == '__main__':
    main()