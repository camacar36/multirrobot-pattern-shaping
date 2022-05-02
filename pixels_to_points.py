#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Polygon, Point32
import tf

def callback(data):
    global pub
    

    rospy.loginfo("New Poligon")
    br = tf.TransformBroadcaster()

    for index, point in enumerate(data.points):
        print(point.x, point.y)
        br.sendTransform((point.y/10, point.x/10, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "point"+str(index),
                     "map")

def pixels_to_points():

    rospy.init_node('pixels_to_points', anonymous=True)

    rospy.Subscriber("/goal_points", Polygon, callback)

    rospy.spin() # keeps python from exiting until this node is stopped


if __name__ == '__main__':
    try:
        pixels_to_points()
    except rospy.ROSInterruptException:
        pass