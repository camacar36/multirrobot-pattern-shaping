import rospy
import tf2_ros
import math
import numpy as np
from bb_utils import Point

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

    # new_robot_points = compute_new_robot_points(m, points, robots)
   

    # for i in range(1,len(points)):
    #     for j in range(1,len(new_robot_points)):
    #         px = points[i].x
    #         py = points[i].y

    #         rx = new_robot_points[j].x
    #         ry = new_robot_points[j].y

    #         m[i, j] = math.sqrt(pow(px-rx, 2) + pow(py - ry, 2))
    

    return path, m

def get_robots():
    robots = [Point(0,0), Point(1,0), Point(2,0), Point(3,0), Point(4,0), Point(5,0), Point(0,1), Point(0,2), Point(0,3), Point(0,4), Point(0,5)]

    # rospy.init_node("bb_algorithm")

    # tfBuffer = tf2_ros.Buffer()
    # listener = tf2_ros.TransformListener(tfBuffer)

    # rate = rospy.Rate(10.0)

    # for i in range(1, 11):
    #     trans = None
    #     while trans == None:
    #         try:

    #             trans = tfBuffer.lookup_transform('map','robot' + str(i) + '_tf/odom', rospy.Time())
    #             robots.append(Point(trans.transform.translation.x, trans.transform.translation.y))
            
    #         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                
    #             rate.sleep()
    #             continue

    

    return robots

def get_points():
    points = [Point(0,0), Point(1,3), Point(2,3), Point(3,3), Point(4,3), Point(5,3), Point(1,2), Point(2,2), Point(3,2), Point(4,2), Point(5,2)]
    return points


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
