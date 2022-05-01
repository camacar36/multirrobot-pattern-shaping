import math
import numpy as np
import time
import rospy

class Node():
   
    
    m = []
    m_opt = []

    def __init__(self, p, r, current_weight, path, sol):
        # self.map = map
        self.p = p
        self.r = r 
        self.current_weight = current_weight
        self.path = path
        
        
        self.pessimistic = self.current_weight + self.greedy()
        self.optimistic = self.current_weight + Node.m_opt[self.p]  #self.opt_greedy()
        self.sol = sol



    def greedy(self):

        aux_path = np.copy(self.path)
        greedy = 0

        for i in range(self.p + 1, len(Node.m)):

            best_val = math.inf
            pos_best = 0
            for j in range(1, len(Node.m[i])):               

                if not aux_path[j]:
                    if Node.m[i, j] < best_val:

                        best_val = Node.m[i, j]
                        pos_best = j
                        
            aux_path[pos_best] = True
            greedy = greedy + best_val

        return greedy

    def greedy2(self):

        aux_path = np.copy(self.path)
        greedy = 0
        for i in range(self.p + 1, len(Node.m)):

            pos_best = 1
            for j in range(1, len(Node.m[i])):               

                if not aux_path[j]:
                    pos_best = j
                    break
                        
            aux_path[pos_best] = True
            greedy = greedy + Node.m[i][pos_best]

        return greedy

    def opt_greedy(self):

        greedy = 0

        for i in range(self.p + 1, len(Node.m)):

            best_val = math.inf
            for j in range(1, len(Node.m[i])):               

                if Node.m[i, j] < best_val:

                    best_val = Node.m[i, j]
                        
            greedy = greedy + best_val

        return greedy

    




class Priority_Queue():
    def __init__(self):
        self.queue = []

    def isEmpty(self):
        return len(self.queue) == 0

    def push(self, node, _key):
        self.queue.append(node)
        self.queue = sorted(self.queue, key=_key)

    def pop(self):
        return self.queue.pop()

    def top(self):        
        return self.queue.pop(0)

class Point():
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)


def compute_matrix(points, robots):
    m = np.zeros((len(points), len(robots)))
    path = [True]
    aux_opt = [0]
    m_opt = []
    for i in range(1,len(points)):
        path.append(False)
        best_val = math.inf
        for j in range(1,len(robots)):
            px = points[i].x
            py = points[i].y

            rx = robots[j].x
            ry = robots[j].y

            m[i, j] = math.sqrt(pow(px-rx, 2) + pow(py - ry, 2))

            if m[i, j] < best_val:

                best_val = m[i, j]

        aux_opt.append(best_val)

    
    for i in range(len(aux_opt)):
        opt = 0
        for k in range(i+1, len(aux_opt)):
            opt = opt + aux_opt[k]
        m_opt.append(opt)

    return path, m, m_opt



