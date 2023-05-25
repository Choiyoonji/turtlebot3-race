from math import sin, cos, pi
import numpy as np
from scipy.spatial import distance

Hp = 1
Gp = 1

MAP_B = 20
MAP_H = 60
TURTLE_SIZE = 1

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
        
    def calc_gcost(self, node):
        if node.pnode:
            node.gcost = node.pnode.gcost-1
        else:
            node.gcost = 0
        return node.gcost
    
    def calc_hcost(self, node):
        node.hcost = MAP_H - node.y
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
                    xd = xy[0] + i
                    yd = xy[1] + j
                    if MAP_B > xd >=0 and MAP_H > yd >= 0:
                        # print(1)
                        self.MAP[xd,yd] = -1
    
    def GetNewNodes(self):
        for i in ind:
            xd = self.curNode.x - i[0]  
            yd = self.curNode.y - i[1]
            if MAP_B > xd >=0 and MAP_H > yd >= 0:
                xd = int(xd)
                yd = int(yd)
                if not self.MAP[xd,yd]:
                    self.MAP[xd,yd] = 2
                    self.open_list.append(Node([xd,yd], self.curNode))
                    self.curNode.cnode.append(Node([xd,yd],self.curNode))
        
    def generate_path(self, obs_xy):
        self.obs = []
        self.obs.extend(obs_xy)
        self.make_map(obs_xy)

        self.open_list = []
        self.close_list = []
        
        self.xy_start = [MAP_B/2, 0]
        
        self.curNode = Node(self.xy_start)
        self.open_list.append(self.curNode)
        
        while self.open_list:
            self.curNode = min(self.open_list, key=lambda n: n.fcost)
            self.close_list.append(self.curNode)
            self.open_list.remove(self.curNode)
            
            if self.curNode.y == MAP_H-1:
                break
            
            self.GetNewNodes()
            
            for n_node in self.open_list:
                self.calc_fcost(n_node)
            
            # print(len(self.open_list),self.curNode.x,self.curNode.y)
                
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
    obs = [[3,10],[4,10],[5,10],[7,10],[8,10],[9,10],[10,10]]
    path = astar.generate_path(obs)
    # ma = np.array(astar.MAP)
    np.set_printoptions(threshold=np.inf,linewidth=np.inf)
    print(path)

if __name__ == '__main__':
    main()