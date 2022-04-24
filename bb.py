import approach
import rospy
from bb_utils import Node, Priority_Queue, Point
import math
import numpy as np
import time
import tf2_ros


def is_feasible(b):
    return b

def is_leaf(node):    
    if node.p == len(node.map) - 1:
        if node.current_weight <= Node.best_v:
            Node.best_v = node.current_weight
            Node.best_sol = np.copy(node.sol)
        return True
    else:
        return False



def bb_algorithm(n0):

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
        for i in range(1, len(node.map)):
            aux_path = np.copy(node.path)

            if not is_feasible(aux_path[i]) :
                aux_path[i] = True
                aux_sol = np.append(node.sol, i)
                n = Node(   node.map, 
                            node.p + 1, 
                            i, 
                            node.current_weight + node.map[node.p + 1, i],
                            aux_path, 
                            aux_sol
                        )
                

                if n.pessimistic < Node.best_v:
                    Node.best_v = n.pessimistic

                

                if n.optimistic > Node.best_v or n.current_weight > Node.best_v :
                    podados = podados + 1
                    continue 
                explorados = explorados + 1
                pq.push(n, lambda n: n.pessimistic)

        # for n in pq.queue:
        #     print(n.r, n.sol, n.path)
        # print("---")
    print(explorados, podados)
        


def main():
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

    

    path, m = approach.compute_matrix(approach.get_points(), approach.get_robots())
    n0 = Node(m, 0, 0, m[0,0], path, [])

    print("holahola")
    print(m)

    # llamar al algoritmo
    start = time.time()
    bb_algorithm(n0)
    stop = time.time()
    print(stop-start)
    print(Node.best_v, Node.best_sol, Node.cont)




main()

