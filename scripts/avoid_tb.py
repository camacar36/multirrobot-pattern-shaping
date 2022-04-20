from cmath import nan
import rospy
import argparse
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
from cv_bridge import CvBridge
import cv2
import time

parser = argparse.ArgumentParser()
parser.add_argument('-robot', type=str,
                    help='robot name')

rosargs = rospy.myargv()
args = parser.parse_args(rosargs[1:])

# rgb and depth image initialization
image = np.zeros((480, 640, 3), np.uint8)
depth_image = np.zeros((480, 640, 1), np.uint8)

# callback function to get rgb image
def read_image(img):
    global image
    image = bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')

# callback function to get depth image
def read_depth_image(img):
    global depth_image
    depth_image = bridge.imgmsg_to_cv2(img, desired_encoding='32FC1')


# function that extracts the pixels where the obstacle may be
# and determines wether to turn left, right or not avoiding  
def analyze_img(img):
    global depth_image

    # convert into hsv colorspace
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # binarize the image parts where the turtlebots are
    mask = cv2.inRange(img, low_threshold, high_threshold)


    cont = 0
    left = 0
    right = 0

    # counting how many pixels are occupied by the turtlebots nearer than 0.6 metres
    for i in range(480):
        for j in range(640):
            if mask[i,j] == 255:
                if depth_image[i,j] < 0.6 and not np.isnan(depth_image[i,j]):
                    cont = cont + 1

                    # if the majority is on the left or on the right side
                    if j < 320:
                        left = left + 1
                    else:
                        right = right + 1

    # if there are more than 10000 pixels it can be considered to need avoidance
    if cont > 10000:
        if left > right:
            side = -1 # turn right because object is on the left
        else:
            side = 1 # turn left because object is on the right
        return True, side
    else:
        return False, None




bridge = CvBridge()

# color image thresholds
high_threshold = np.array([50, 50, 55],  np.uint8)
low_threshold = np.array([0, 0, 0],  np.uint8)

# movement to be done
mov = Twist()
mov.linear.x = 0
mov.angular.z = 0.3

rospy.init_node('avoid_tb')

sub_cam = rospy.Subscriber('/' + args.robot + '/camera/rgb/image_raw', Image, read_image)
sub_depth_cam = rospy.Subscriber('/' + args.robot + '/camera/depth/image_raw', Image, read_depth_image)
pub_mov = rospy.Publisher('/' + args.robot + '/mobile_base/commands/velocity', Twist, queue_size=1)


while not rospy.is_shutdown():
    avoid, side = analyze_img(image)
    if avoid:
        mov.angular.z = 0.3 * side
        pub_mov.publish(mov)


