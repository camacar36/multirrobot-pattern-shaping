#!/usr/bin/env python
from Robot import Robot
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Polygon
from std_msgs.msg import UInt16MultiArray

import rospy
import sys
import tf
import math

def main():

    if len(sys.argv) != 4:
        raise ValueError("Incorrect number of arguments (1 number)")
    elif isinstance(int(sys.argv[1]), int) == False:
        raise ValueError("Argument must be an integer")

    robot_number = sys.argv[1]
    
    # Creation of first robot
    robot1 = Robot(robot_number)

    while not rospy.is_shutdown():

        goal_points = rospy.wait_for_message('/goal_points', Polygon)
        print("hola")
        final_goal_points = rospy.wait_for_message('/final_goal_points', UInt16MultiArray)

        index = final_goal_points.data.index(int(robot_number))

        point = [goal_points.points[index].x, goal_points.points[index].y]

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
        while actual_distance > 0.1:

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


if __name__ == "__main__":
    main()