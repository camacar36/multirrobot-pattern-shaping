#!/usr/bin/env python
from Robot import Robot
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Polygon
from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import String

import rospy
import sys
import tf
import math

point_check = False
checked_index = []
index = -1

def callback(data):
    global point_check
    global checked_index
    global index

    checked_index.append(data)

    if data == str(index):
        point_check = True

    

def main():

    global point_check
    global checked_index
    global index

    if len(sys.argv) != 4:
        raise ValueError("Incorrect number of arguments (1 number)")
    elif isinstance(int(sys.argv[1]), int) == False:
        raise ValueError("Argument must be an integer")

    robot_number = sys.argv[1]
    
    # Creation of first robot
    robot1 = Robot(robot_number)

    sub = rospy.Subscriber('/goal_points_check', String, callback)
    pub = rospy.Publisher('/goal_points_check', String, queue_size=10)

    while not rospy.is_shutdown():

        if not point_check: 
            index = -1
            goal_points = rospy.wait_for_message('/goal_points', Polygon)
        
        point_check = False

        print("hola")
        
        print(goal_points)

        dist = None
        point = None

        # Get closest point
        for ite in range(len(goal_points.points)):
            print("ite: " + str(ite))
            
            for checked in checked_index:
                if ite == checked:
                    break

            if dist == None:
                point = [goal_points.points[ite].x, goal_points.points[ite].y]
                dist = robot1.get_distance_to_objective(point)
                index = ite
            else:
                ite_point = [goal_points.points[ite].x, goal_points.points[ite].y]
                ite_dist = robot1.get_distance_to_objective(ite_point)
                print("ite_dist: " + str(ite_dist))
                if ite_dist < dist:
                    point = ite_point
                    dist = ite_dist
                    index = ite

            print("index: " + str(index))
            print("dist: " + str(dist))
            print(point)
            print()

        print(point)

        # Setting initial position and orientation of the robot
        robot1.get_tf()


        # --- Previous orientation of the robot ---

        vel = 0.3

        # Orientate the robot to the desired point
        robot1.face_to_point(point,vel)


        # --- Movement of the robot ---

        actual_distance = robot1.get_distance_to_objective(point)
        prev_distance = actual_distance

        # While distance to the final point is more than 0.1
        while actual_distance > 0.1 and not point_check:

            # Move forward
            robot1.get_tf()
            actual_distance = robot1.get_distance_to_objective(point)
            robot1.move_forward(vel)

            detected, side = robot1.detect_obstacles()
            if detected:
                if side == 1: # Turn left
                    start_angle = robot1.get_actual_orientation()
                    while detected == True:
                        detected, side = robot1.detect_obstacles()
                        robot1.turn_left(vel)
                
                elif side == -1: # Turn right
                    start_angle = robot1.get_actual_orientation()
                    while detected == True:
                        detected, side = robot1.detect_obstacles()
                        robot1.turn_right(vel)

            # If the previous distance is lower than actual distance, it means that the robot have passed near the point
            # but not so close enough to stop (>0.1), so it reorientates robot and keeps moving
            if prev_distance < actual_distance:
                robot1.stop()
                robot1.face_to_point(point,vel)

            prev_distance = actual_distance

    
        robot1.stop()
        
        if not point_check:
            # pub point check
            pub.publish(str(index))




if __name__ == "__main__":
    main()