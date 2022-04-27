#!/usr/bin/python
import rospy
import argparse
from geometry_msgs.msg import Twist
# from features import Point
import math
import tf2_ros
import time
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Polygon

class Point: 
    def __init__(self, x, y):
        self.x = x
        self.y = y

parser = argparse.ArgumentParser()
parser.add_argument('-robot', type=str,
                    help='robot name')
parser.add_argument('-x', type=str)
parser.add_argument('-y', type=str)
parser.add_argument('-z', type=str)
parser.add_argument('-R', type=str)
parser.add_argument('-P', type=str)
parser.add_argument('-Y', type=str)

rosargs = rospy.myargv()
args = parser.parse_args(rosargs[1:])

points = []
odom = Odometry()

def compute_angle(r, p):
    diff_x = p.x - r.x
    diff_y = p.y - r.y
    return math.atan2(diff_y, diff_x)

def compute_dist(r, p):    
    return math.sqrt(pow(r.x - p.x, 2) + pow(r.y - p.y, 2))

def get_init_robot_pos():
    # tfBuffer = tf2_ros.Buffer()
    # listener = tf2_ros.TransformListener(tfBuffer)

    # rate = rospy.Rate(10.0)

   
    # trans = None
    # while trans == None:
    #     try:

    #         trans = tfBuffer.lookup_transform('map', 'robot' + str(args.robot) + '_tf/odom', rospy.Time())
        
    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            
    #         rate.sleep()
    #         continue


    # x = trans.transform.translation.x
    # y = trans.transform.translation.y

    # q = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
    # _,_,yaw = euler_from_quaternion(q)

    x = float(args.x)
    y = float(args.y)
    yaw = float(args.Y)
    return Point(x, y), yaw

def get_robot_transf(init_pos,yaw_1):
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    q = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
    _,_,yaw = euler_from_quaternion(q)
    return Point(x + init_pos.x, y + init_pos.y), yaw + yaw_1


def get_points(msg):
    global points
    points = msg.points
    print("hola")

def get_odom(msg):
    global odom
    odom = msg



rospy.init_node('move_tb_robot' + str(args.robot))


pub_mov = rospy.Publisher('/robot' + str(args.robot) + '/mobile_base/commands/velocity', Twist, queue_size=5)
sub_odom = rospy.Subscriber('/robot' + str(args.robot) + '/odom', Odometry, get_odom)
sub_point = rospy.Subscriber('/goal_points', Polygon, get_points)


init_pos,yaw = get_init_robot_pos()



while not rospy.is_shutdown():
    
    try:
        r,yaw = get_robot_transf(init_pos,yaw)
        p = points[int(args.robot)-1]
        print(args.robot, p)
        mov = Twist()
        mov.angular.z = 0
        mov.linear.x = 0
        vel_z = 0.2

        ang = compute_angle(r, p)
        dist = compute_dist(r, p)
        if abs(ang - abs(yaw)) > 0.1:
            if ang-yaw > 0:
                mov.angular.z = -vel_z
            else:
                mov.angular.z = vel_z
        else:
            if dist > 0.05:
                mov.linear.x = 0.5
        # print(ang-r.z, r.z, mov,dist)
        pub_mov.publish(mov)
        time.sleep(0.1)
    except:
        continue
