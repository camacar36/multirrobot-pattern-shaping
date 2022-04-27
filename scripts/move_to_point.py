from Robot import Robot
from nav_msgs.msg import Odometry

import rospy
import sys


def main():

    if len(sys.argv) != 2:
        raise ValueError("Incorrect number of arguments (1 number)")
    elif isinstance(int(sys.argv[1]), int) == False:
        raise ValueError("Argument must be an integer")

    number = sys.argv[1]

    vel_topic = '/robot' + number + '/mobile_base/commands/velocity'
    odom_topic = '/robot' + number + '/odom'
    
    # Creation of first robot
    init_pos_rob1 = [0, 0]  
    #robot1 = Robot(init_pos_rob1, '/robot1/mobile_base/commands/velocity')
    robot1 = Robot(init_pos_rob1, vel_topic)
    # odom_topic_rob1 = rospy.Subscriber('/robot1/odom', Odometry, robot1.odom_callback)

    # Final point to reach
    point = [2,2]

    # Settimg odometry
    #robot1.get_odom('/robot1/odom')
    robot1.get_odom(odom_topic)

    # Calculate the orientation difference between the robot and the final point
    orientation_difference = robot1.get_difference_angle(point) - robot1.get_actual_orientation()

    # --- Previous orientation of the robot ---

    # Orientate the robot with +-0.025 error
    while abs(orientation_difference) > 0.05:
        robot1.get_odom(odom_topic)
        orientation_difference = robot1.get_difference_angle(point) - robot1.get_actual_orientation()
        robot1.turn_left(0.1) # Robot turns to left

    # --- Movement of the robot ---
    robot1.get_odom(odom_topic)
    actual_pos = robot1.get_actual_position()
    prev_pos = actual_pos

    # While distance to the final point is more than 0.1
    while robot1.get_distance_to_objective(point) > 0.1:

        # Move forward
        robot1.get_odom(odom_topic)
        actual_pos = robot1.get_actual_position()
        robot1.move_forward(0.4)

        # If the previous distance is lower than actual distance, it means that the robot have passed near the point
        # but not so close enough to stop (>0.1), so it reorientates robot and keeps moving
        if prev_pos < actual_pos:

            # Calculates orientatios difference
            orientation_difference = robot1.get_difference_angle(point) - robot1.get_actual_orientation()

            # Rotates until it looks to the point
            while abs(orientation_difference) > 0.2:
                robot1.get_odom(odom_topic)
                orientation_difference = robot1.get_difference_angle(point) - robot1.get_actual_orientation()
                robot1.turn_left(0.5)

        prev_pos = actual_pos

    robot1.stop()


if __name__ == "__main__":
    main()
