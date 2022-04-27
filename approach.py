import rospy
import tf2_ros
import math
import numpy as np
from bb_utils import Point




def compute_new_robot_points(m, points, robots):

    p = []
    good_one = 0
    for j in range(1, len(m)):
        menor = math.inf
        
        for i in range(1, len(m[j])):
            if m[i, j] < menor:
                menor = m[i, j]
                good_one = i
        p.append(good_one)
    print(p)
    new_robots = np.copy(robots)

    for i in range(len(p)):
        newp = points[p[i]]
        r = robots[i+1]

        angulo = math.atan2(newp.y - r.y, newp.x - r.x)
        dist = math.sqrt(pow(newp.y - r.y, 2) + pow(newp.x - r.x, 2))
        
        # puede darse el caso de que la distancia sea menor de 1 y no vale la pena
        if dist > 1:
            nd = dist - 1
            nx = r.x + nd * math.cos(angulo)
            ny = r.y + nd * math.sin(angulo)
            new_robots[i+1] = Point(nx, ny)


        print('(',newp.x, ',', newp.y,')(', r.x, ',', r.y,')', '-->', dist)
        new_dist = math.sqrt(pow(points[p[i]].y - new_robots[i+1].y, 2) + pow(points[p[i]].x - new_robots[i+1].x, 2))
        print("-")
        print('(',points[p[i]].x, ',', points[p[i]].y,')(', new_robots[i+1].x, ',', new_robots[i+1].y,')', '-->', new_dist)
        print("------------")


    
    

    


    return new_robots
