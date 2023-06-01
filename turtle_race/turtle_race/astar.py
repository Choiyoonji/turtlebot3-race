from math import sin, cos, pi
import numpy as np
from scipy.spatial import distance
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

Hp = 1
Gp = 1

MAP_B = 300
MAP_H = 600
TURTLE_SIZE = 5

ind = [[0,1],[1,0],[-1,0],[0,-1], [1,1], [-1,1], [1,-1], [-1,-1]]

class Node:
    def __init__(self, xy, pnode = None):
        self.pnode = pnode
        self.cnode = []
        
        self.x = xy[0]
        self.y = xy[1]
        
        self.gcost = 0
        self.hcost = 0
        self.fcost = 0

class Astar:
    def __init__(self):
        self.MAP = np.zeros([MAP_B,MAP_H])
        
        self.open_list = []
        self.close_list = []
        self.goal = MAP_H - 1
        
    def calc_gcost(self, node):
        if node.pnode:
            node.gcost = node.pnode.gcost-1
        else:
            node.gcost = 0
        return node.gcost
    
    def calc_hcost(self, node):
        node.hcost = self.goal - node.y
        return node.hcost
    
    def calc_fcost(self, node):
        node.fcost = Gp * self.calc_gcost(node) + Hp * self.calc_hcost(node)
        if self.MAP[node.x][node.y] == 1:
            node.fcost = 9999999999999
        return node.fcost
    
    def make_map(self,obs_xy):
        for xy in obs_xy:
            for i in range(-TURTLE_SIZE,TURTLE_SIZE):
                for j in range(-TURTLE_SIZE,TURTLE_SIZE):
                    xd = int(xy[0] + i)
                    yd = int(xy[1] + j)
                    if MAP_B > xd >=0 and MAP_H > yd >= 0:
                        # print(xd,yd)
                        self.MAP[xd,yd] = -1
    
    def decide_goal(self):
        deq = deque([])
        [i,j] = self.xy_start
        self.MAP[i,j] = 5
        MaxY = j
        
        for k in range(len(ind)):
            deq.append([i+ind[k][0],j+ind[k][1]])
        while True:
            if len(deq) == 0:
                break
            p = deq.popleft()
            x = p[0]
            y = p[1]
            
            if x < 0 or y < 0 or x >= MAP_B or y >= MAP_H:
                continue
            if self.MAP[x,y] == 0:
                self.MAP[x,y] = 5
                if MaxY < y:
                    MaxY = y
                for k in range(len(ind)):
                    deq.append([x+ind[k][0],y+ind[k][1]])
                    
        return MaxY
    
    def GetNewNodes(self):
        for i in ind:
            xd = self.curNode.x - i[0]  
            yd = self.curNode.y - i[1]
            if MAP_B > xd >=0 and self.goal >= yd >= 0:
                xd = int(xd)
                yd = int(yd)
                if self.MAP[xd,yd] == 5:
                    self.MAP[xd,yd] = 2
                    self.open_list.append(Node([xd,yd], self.curNode))
                    self.curNode.cnode.append(Node([xd,yd],self.curNode))
        
    def generate_path(self, obs_xy):
        self.obs = []
        obs_xy = np.delete(obs_xy,np.where(obs_xy[:,1]<0),0)
        self.obs.extend(obs_xy)
        self.make_map(obs_xy)
        self.xy_start = [int(MAP_B/2), 0]
        self.goal = self.decide_goal()
        # print(self.goal)

        self.open_list = []
        self.close_list = []
        
        self.curNode = Node(self.xy_start)
        self.open_list.append(self.curNode)
        
        while self.open_list:
            self.curNode = min(self.open_list, key=lambda n: n.fcost)
            self.close_list.append(self.curNode)
            self.open_list.remove(self.curNode)
            
            if self.curNode.y == self.goal:
                break
            
            self.GetNewNodes()
            
            for n_node in self.open_list:
                self.calc_fcost(n_node)
                
        return self.extracted_path()

    def extracted_path(self):
        path = [[self.curNode.x,self.curNode.y]]
        self.curNode = self.curNode.pnode
        
        while True:
            path.append([self.curNode.x,self.curNode.y])
            
            if self.curNode.x == self.xy_start[0] and self.curNode.y == self.xy_start[1]:
                break
            
            self.curNode = self.curNode.pnode
            
        return path

def main():
    astar = Astar()
    obs = [[1,10],[2,10],[3,10],[4,10],[5,10],[7,10],[8,10],[9,10],[10,10],[11,10],[12,10],[13,10],[14,10],[15,10],[16,10],[17,10],[18,10],[19,10],[0,10]]
    path = astar.generate_path(obs)
    np.set_printoptions(threshold=np.inf,linewidth=np.inf)
    ma = np.array(astar.MAP)
    print(ma)
    print(path)

if __name__ == '__main__':
    main()