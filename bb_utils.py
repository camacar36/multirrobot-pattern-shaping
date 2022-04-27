import math
import numpy as np
import time
import rospy

class Node():
    cont = 0
    best_v = math.inf
    best_sol = []

    def __init__(self, map, p, r, current_weight, path, sol):
        self.map = map
        self.p = p
        self.r = r 
        self.current_weight = current_weight
        self.path = path
        
        start = time.time()
        self.pessimistic = self.current_weight + self.greedy()
        self.optimistic = self.current_weight + self.opt_greedy()
        end = time.time()
        Node.cont = Node.cont + end - start
        self.sol = sol

    def greedy(self):

        aux_path = np.copy(self.path)
        greedy = 0

        for i in range(self.p + 1, len(self.map)):

            best_val = math.inf
            pos_best = 0
            for j in range(1, len(self.map[i])):               

                if not aux_path[j]:
                    if self.map[i, j] < best_val:

                        best_val = self.map[i, j]
                        pos_best = j
                        
            aux_path[pos_best] = True
            greedy = greedy + best_val

        return greedy

    def opt_greedy(self):

        greedy = 0

        for i in range(self.p + 1, len(self.map)):

            best_val = math.inf
            for j in range(1, len(self.map[i])):               

                if self.map[i, j] < best_val:

                    best_val = self.map[i, j]
                        
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
        self.x = x
        self.y = y


def compute_matrix(points, robots):
    m = np.zeros((len(points), len(robots)))
    path = [True]
    for i in range(1,len(points)):
        path.append(False)
        for j in range(1,len(robots)):
            px = points[i].x
            py = points[i].y

            rx = robots[j].x
            ry = robots[j].y

            m[i, j] = math.sqrt(pow(px-rx, 2) + pow(py - ry, 2))

    return path, m



