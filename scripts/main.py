from Robot import Robot
from nav_msgs.msg import Odometry

import rospy
import sys
import tf
import math


def main():

    if len(sys.argv) != 2:
        raise ValueError("Incorrect number of arguments (1 number)")
    elif isinstance(int(sys.argv[1]), int) == False:
        raise ValueError("Argument must be an integer")

    robot_number = sys.argv[1]
    
    # Creation of first robot
    robot1 = Robot(robot_number)

    # Final point to reach
    point = [-1,-math.sqrt(3)]

    # Settimg initial position and orientation of the robot
    robot1.get_tf()


    # --- Previous orientation of the robot ---

    # Orientate the robot to the desired point
    robot1.face_to_point(point,0.1)


    # --- Movement of the robot ---

    actual_distance = robot1.get_distance_to_objective(point)
    prev_distance = actual_pos

    # While distance to the final point is more than 0.1
    while actual_distance > 0.1:

        # Move forward
        robot1.get_tf()
        actual_distance = robot1.get_distance_to_objective(point)
        robot1.move_forward(0.4)

        # If the previous distance is lower than actual distance, it means that the robot have passed near the point
        # but not so close enough to stop (>0.1), so it reorientates robot and keeps moving
        if prev_pos < actual_pos:
            robot1.stop()
            robot1.face_to_point(point,0.1)

        prev_distance = actual_pos

    robot1.stop()


if __name__ == "__main__":
    main()
