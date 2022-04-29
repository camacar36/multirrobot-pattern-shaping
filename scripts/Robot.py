#!/usr/bin/env python

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import geometry_msgs.msg
import rospy
import math
import tf

class Robot:

    # Initialization of the node, setting of the velocities and suscription to the velocity node
    def __init__(self, robot_number = '1'):

        # Creation of the robot velocity topic name
        vel_topic = '/robot' + robot_number + '/mobile_base/commands/velocity'
        self.tf_topic = '/robot' + robot_number + '_tf/base_footprint'
        self.odom_topic = '/robot' + robot_number + '/odom'

        # Initialization of the node
        robot_name = 'robot' + robot_number
        rospy.init_node(robot_name)

        # Creation of listener
        self.Listener = tf.TransformListener()  
        m = geometry_msgs.msg.TransformStamped()
        m.header.frame_id = '/map'
        m.child_frame_id = robot_name + '/base_footprint'
        self.Listener.setTransform(m)

        # Initialization of initial pose
        trans = None
        while trans == None:
            try:
                trans, rot = self.Listener.lookupTransform('/map', self.tf_topic, rospy.Time.now())
            except:
                continue
        self.init_pos = [trans[0], trans[1]]
        self.actual_pos = [trans[0], trans[1]]
        self.actual_orientation = 0

        # THIS IS JUST FOR DEBUGGING
        self.reached = False  

        # Publishing to the topic to control velocity
        self.vel_topic = rospy.Publisher(vel_topic, Twist, queue_size=1)

        # Setting rate
        #self.rate = rospy.Rate(5)
        self.rate = rospy.Rate(1)


    # Return actual position
    def get_actual_position(self):
        return self.actual_pos


    # Return actual orientation
    def get_actual_orientation(self):
        return self.actual_orientation


    # Return variable for debugging
    def get_reached(self):
        return self.reached


    # Set variable for debugging
    def set_reached(self, value):
        self.reached = value


    # Callback to get odometry information to set position and orientation
    # from a subscriber node that reads data continuously
    def odom_callback(self, data):

        # Take position
        self.actual_pos[0] = round(data.pose.pose.position.x,3) + self.init_pos[0]
        self.actual_pos[1] = round(data.pose.pose.position.y,3) + self.init_pos[1]

        # Take and process orientation quaternion
        orientation = data.pose.pose.orientation
        a1 = 2*(orientation.w*orientation.z+orientation.x*orientation.y)
        a2 = 1-2*(orientation.y*orientation.y+orientation.z*orientation.z)
        self.actual_orientation = math.atan2(a1,a2)


    # Gets the position and orientation of the robot using odometry only when this function is called
    def get_odom(self):
        data = rospy.wait_for_message(self.odom_topic, Odometry)

        # Take position
        self.actual_pos[0] = round(data.pose.pose.position.x,3) + self.init_pos[0]
        self.actual_pos[1] = round(data.pose.pose.position.y,3) + self.init_pos[1]

        # Take and process orientation quaternion
        orientation = data.pose.pose.orientation
        a1 = 2*(orientation.w*orientation.z+orientation.x*orientation.y)
        a2 = 1-2*(orientation.y*orientation.y+orientation.z*orientation.z)
        self.actual_orientation = math.atan2(a1,a2)


    # Reads and saves actual position and orientation using transformations between the 
    # robot and the world only when this function is called
    def get_tf(self):
        trans = None
        
        while trans == None:
            try:
                trans, rot = self.Listener.lookupTransform('/map', self.tf_topic, rospy.Time.now())
            except:
                continue

        # Take actual position
        self.actual_pos[0] = round(trans[0], 3)
        self.actual_pos[1] = round(trans[1], 3)

        # Take and process orientation quaternion
        a1 = 2*(rot[3]*rot[2]+rot[0]*rot[1])
        a2 = 1-2*(rot[1]*rot[1]+rot[2]*rot[2])
        self.actual_orientation = math.atan2(a1,a2)


    # Move forward
    def move_forward(self, velocity):
        twist_rob = Twist()
        twist_rob.linear.x = velocity
        twist_rob.angular.z = 0.0
        self.vel_topic.publish(twist_rob)


    # Moves the robot forward until it is close enough to the final point
    def move_forward_until(self, distance, velocity):
        actual_dist = self.get_distance_to_objective(point)
        while actual_dist > distance:
            self.move_forward(velocity)


    # Turn right at a certain velocity
    def turn_right(self, velocity):
        twist_rob = Twist()
        twist_rob.linear.x = 0.0
        twist_rob.angular.z = -velocity
        self.vel_topic.publish(twist_rob)


    # Turn left at a certain velocity
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


    # Get the angle between robot and final point
    def get_difference_angle(self, point):

        diff_x = point[0] - self.actual_pos[0]
        diff_y = point[1] - self.actual_pos[1]
        angle = 0
        if diff_x != 0 and diff_y != 0:
            angle = math.atan2(diff_y,diff_x)
        else:
            if diff_x == 0:
                if diff_y < 0:
                    angle = (3/2)*math.pi
                elif diff_y > 0: 
                    angle = (1/2)*math.pi
            elif diff_y == 0:
                if diff_x < 0:
                    angle = math.pi
                elif diff_x > 0: 
                    angle = 0
        return angle


    # Return distance between actual position and end position
    def get_distance_to_objective(self, point):
        diff_x = point[0] - self.actual_pos[0]
        diff_y = point[1] - self.actual_pos[1]
        return math.sqrt(pow(diff_x,2) + pow(diff_y,2))


    # Faces the robot to the desired point with vel/10 radians of error
    def face_to_point(self, point, velocity):
        
        # This while True is in fact a do-while structure, the condition to break is inside
        while True:
            self.get_tf()
            final_angle = self.get_difference_angle(point)
            actual_angle = self.actual_orientation

            # Orientation of the robot goes from 0 to pi and then from -pi to 0, so these
            # 2 conditions convert the orientation so it goes from 0 to 2*pi
            if final_angle < 0:
                final_angle += 2*math.pi

            if actual_angle < 0:
                actual_angle += 2*math.pi

            # Calculates the difference between the actual orientation adn the desired one
            orientation_difference = abs(final_angle - actual_angle)

            # Condition to break the do-while loop
            if orientation_difference < velocity/10:
                break   
            
            # Depending of the actual orientation of the robot and the desired final point, 
            # it will rotate left or right.
            # If the angle of the actual orientation is less than the desired one, it will turn right or 
            # left depending if the angle difference is less or more than pi
            if actual_angle < final_angle:
                if orientation_difference < math.pi:
                    self.turn_left(velocity)
                else:
                    self.turn_right(velocity)
            # If the angle of the actual orientation is more than the desired one, the conditions for
            # right and left turns are reversed
            else:
                if orientation_difference < math.pi:
                    self.turn_right(velocity)
                else:
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




    
