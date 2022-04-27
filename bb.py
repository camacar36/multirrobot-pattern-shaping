from geometry_msgs.msg import Polygon, Point32
from approach import *
import rospy
from bb_utils import *
import math
import numpy as np
import time
import tf2_ros

points  = []
start   = ""
stop    = ""

def is_feasible(b):
    return b

def is_leaf(node):    
    if node.p == len(Node.m) - 1:
        if node.current_weight <= Node.best_v:
            Node.best_v = node.current_weight
            Node.best_sol = np.copy(node.sol)
            # print("best:", Node.best_sol, node.sol)
        return True
    else:
        return False



def bb_algorithm(n0):
    global start, stop

    start = time.time()

    pq = Priority_Queue()
    pq.push(n0, lambda n0 : n0.pessimistic)
    podados = 0
    explorados = 1

    
    while not pq.isEmpty():
        # se obtiene el
        # mejor elemento de la cola y se elimina
        node = pq.top()
        
        if is_leaf(node):  
            continue   
            
        # para cada robot
        for i in range(1, len(Node.m)):
            aux_path = np.copy(node.path)

            if not is_feasible(aux_path[i]) :
                aux_path[i] = True
                aux_sol = np.append(node.sol, i)
                n = Node(   node.p + 1, 
                            i, 
                            node.current_weight + Node.m[node.p + 1, i],
                            aux_path, 
                            aux_sol
                        )
                #print("-",node.sol, n.sol)
                if n.pessimistic < Node.best_v:
                    Node.best_v = n.pessimistic

                if n.optimistic > Node.best_v or n.current_weight > Node.best_v or n.optimistic > n.pessimistic:
                    podados = podados + 1
                    continue 
                explorados = explorados + 1
                pq.push(n, lambda n: n.pessimistic - n.optimistic)

        # for n in pq.queue:
        #     print(n.r, n.sol, n.path)
        # print("---")
    print(explorados, podados)

    stop = time.time()
    print(stop-start)

    print("sol: ", Node.best_sol, Node.best_v)


# def main():
    # leer los datos del problema (robots y puntos)
    # points = [Point(0,0), Point(1,1), Point(2,2), Point(3,3), Point(4,4), Point(5,5)]
    # robots = [Point(0,0), Point(0, 0), Point(1, 2), Point(5,3), Point(1, 1), Point(3, 3)]    

    # points = [Point(0,0), Point(1,3), Point(2,3), Point(3,3), Point(4,3), Point(5,3), Point(1,2), Point(2,2), Point(3,2), Point(4,2), Point(5,2)]
    # robots = [Point(0,0), Point(5, 1), Point(4, 1), Point(3,1), Point(2, 1), Point(1, 1), Point(5,5), Point(4,5), Point(3,5), Point(2,5), Point(1,5)]    

    # points = [Point(0,0), Point(1,3), Point(2,3), Point(1,2), Point(2,2)]
    # robots = [Point(0,0), Point(2,1), Point(1,1), Point(2,4), Point(1,4)]    


    # for k in range(30):
    # points = [Point(0,0)]
    # robots = [Point(0,0)]    



    # for i in range(10):
    #     points.append(Point(np.random.randint(100), np.random.randint(100)))
    #     robots.append(Point(np.random.randint(100), np.random.randint(100)))

def get_points(msg):
    global points
    points.append(Point(0,0))
    for m in msg.points:
        points.append(Point(round(m.x, 2), round(m.y, 2)))

def get_robots():
    robots = [Point(0,0)]

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)

    for i in range(1, len(points)):
        trans = None
        while trans == None:
            try:

                trans = tfBuffer.lookup_transform('map','robot' + str(i) + '_tf/odom', rospy.Time())
                robots.append(Point(trans.transform.translation.x, trans.transform.translation.y))
            
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                
                rate.sleep()
                continue
    
    return robots
    
rospy.init_node("bb_algorithm")

# sub_points = rospy.Subscriber('/goal_points', Polygon, get_points)

# while True:

#     if len(points) > 0:
points = [Point(0,0), Point(1,0.6), Point(1.4,0.6), Point(1.8,0.6), Point(2.4,0.6), Point(2.8,0.6), Point(3.2,0.6), Point(3.8,0.6), Point(4.2,0.6)]#, Point(4.6,0.6)]#, Point(5,0.6)]       
robots = get_robots()
# robots = [Point(0,0), Point(1, 0), Point(2, 0), Point(3,0), Point(4, 0), Point(5, 0), Point(0,1), Point(0,2), Point(0,3)]#, Point(0,4)]#, Point(0,5)]    

for i in range(len(points)):
    print(robots[i].x, robots[i].y, points[i].x, points[i].y)
path, Node.m, Node.m_opt = compute_matrix(points, robots)
print(Node.m_opt)
n0 = Node(0, 0, Node.m[0,0], path, [])


# llamar al algoritmor al algoritmo
bb_algorithm(n0)

# #points = Polygon().points
# points = []

# break





