#!/usr/bin/env python

from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import rospy
import math

class Robot:

    # Initialization of the node, setting of the velocities and suscription to the velocity node
    def __init__(self, init_pos = [0, 0], topic_vel = '/robot1/mobile_base/commands/velocity'):

        # Initialization of the node
        rospy.init_node('robot')

        # Initialization of initial pose
        self.init_pos = init_pos
        self.actual_pos = init_pos
        self.actual_orientation = 0

        # THIS IS JUST FOR DEBUGGING
        self.reached = False      

        # Publishing to the topic to control velocity
        self.vel_topic = rospy.Publisher(topic_vel, Twist, queue_size=1)

        # Setting rate
        #self.rate = rospy.Rate(5)
        self.rate = rospy.Rate(1)
    
    # Callback to get odometry information
    def odom_callback(self, data):

        # Take position
        self.actual_pos[0] = round(data.pose.pose.position.x,3) + self.init_pos[0]
        self.actual_pos[1] = round(data.pose.pose.position.y,3) + self.init_pos[1]

        # Take and process orientation quaternion
        orientation = data.pose.pose.orientation
        a1 = 2*(orientation.w*orientation.z+orientation.x*orientation.y)
        a2 = 1-2*(orientation.y*orientation.y+orientation.z*orientation.z)
        self.actual_orientation = math.atan2(a1,a2)

    def get_odom(self, topic = '/robot1/odom'):
        data = rospy.wait_for_message(topic, Odometry)

        # Take position
        self.actual_pos[0] = round(data.pose.pose.position.x,3)
        self.actual_pos[1] = round(data.pose.pose.position.y,3)

        # Take and process orientation quaternion
        orientation = data.pose.pose.orientation
        a1 = 2*(orientation.w*orientation.z+orientation.x*orientation.y)
        a2 = 1-2*(orientation.y*orientation.y+orientation.z*orientation.z)
        self.actual_orientation = math.atan2(a1,a2)

    # Return actual position
    def get_actual_position(self):
        return self.actual_pos

    def get_actual_orientation(self):
        return self.actual_orientation

    # Move forward
    def move_forward(self, velocity):
        twist_rob = Twist()
        twist_rob.linear.x = velocity
        twist_rob.angular.z = 0.0
        self.vel_topic.publish(twist_rob)

    # Turn right depending on the velocity set at the creation of the object
    def turn_right(self, velocity):
        twist_rob = Twist()
        twist_rob.linear.x = 0.0
        twist_rob.angular.z = -velocity
        self.vel_topic.publish(twist_rob)

    # Turn left depending on the velocity set at the creation of the object
    def turn_left(self, velocity):
        twist_rob = Twist()
        twist_rob.linear.x = 0.0
        twist_rob.angular.z = velocity
        self.vel_topic.publish(twist_rob)

    # Stops the robot movement
    def stop(self):
        twist_rob = Twist()
        twist_rob.linear.x = 0.0
        twist_rob.angular.z = 0.0
        self.vel_topic.publish(twist_rob)

    # Return variable for debugging
    def get_reached(self):
        return self.reached

    # Set variable for debugging
    def set_reached(self, value):
        self.reached = value

    # Get the angle between robot and final point
    def get_difference_angle(self, point):

        diff_x = point[0] - self.actual_pos[0]
        diff_y = point[1] - self.actual_pos[1]
        if diff_x != 0:
            angle = math.atan(diff_y/diff_x)

        if diff_y < 0:
            return angle + math.pi
        return angle
    
    # Return distance between actual position and end position
    def get_distance_to_objective(self, point):
        diff_x = point[0] - self.actual_pos[0]
        diff_y = point[1] - self.actual_pos[1]
        return math.sqrt(pow(diff_x,2) + pow(diff_y,2))


    # Function thats make the robot move to a certain point in the world
    # def move_to_point(self, point):       
    #     while self.get_distance_to_objective(point) > 0.1:
    #         self.move_forward(0.3)    

    #         end_orientation = self.get_difference_angle(point)
    #         orientation_difference = (end_orientation + math.pi) - self.actual_orientation

    #         while abs(orientation_difference) > 0.2:
    #             orientation_difference = (end_orientation) - self.actual_orientation
    #             print("Here")
    #             print(end_orientation)
    #             print(self.actual_orientation)
    #             print(orientation_difference)
    #             if self.actual_orientation > end_orientation and self.actual_orientation < end_orientation + math.pi:
    #                 self.turn_right(0.1)
    #             else:
    #                 self.turn_left(0.1)



            #total_angle = self.get_difference_angle(point)# + self.actual_orientation
            #while total_angle > 0.1 or total_angle < -0.1:
            #    end_angle = self.get_difference_angle(point)
            #    print(self.actual_orientation)
            #    total_angle = self.get_difference_angle(point) - self.actual_orientation
            #    if total_angle > 0.1:
            #        self.turn_right(0.1)
                #elif total_angle < -0.1:
                #    self.turn_left(0.1)
            #    else:
            #        self.stop()
            #self.rate.sleep()

    # Faces the robot to the desired point with +-0.1 radians of error
    def face_to_point(self, point, velocity):
        orientation_difference = self.get_difference_angle(point) - self.actual_orientation
        while abs(orientation_difference) > 0.2:
            orientation_difference = self.get_difference_angle(point) - self.actual_orientation
            self.turn_left(velocity)

    # Moves the robot to the desired point
    def move_to_point(self,point):

        self.face_to_point(point, 0.1)

        actual_dist = self.get_distance_to_objective(point)
        prev_dist = actual_dist
        while actual_dist > 0.1:
            actual_dist = self.get_distance_to_objective(point)
            self.move_forward(0.1)
            # if(actual_dist > prev_dist):
            #     self.face_to_point(point, 0.5)

    # Moves the robot forward until it is close enough to the final point
    def move_forward_until(self, distance, velocity):
        actual_dist = self.get_distance_to_objective(point)
        while actual_dist > distance:
            self.move_forward(velocity)

    