#!/usr/bin/env python

# The car has no odometry published by default
# However, there is a "ground truth" pose, published by a Gazebo plugin.
# Let's just use this for now. 
# So we'll take the values from this pose, and then
# publish a tf between base_link and odom frames. 
# Then we can feed this into the gmapping node. 

import rospy
from nav_msgs.msg import Odometry
import tf
from math import sqrt, pow, atan2
from math import acos
from math import sqrt
from math import pi

from tf.transformations import quaternion_from_euler, euler_from_quaternion

#------------------------------------------
# from https://stackoverflow.com/a/31735880 
def length(v):
    return sqrt(v[0]**2+v[1]**2)
def dot_product(v,w):
   return v[0]*w[0]+v[1]*w[1]
def determinant(v,w):
   return v[0]*w[1]-v[1]*w[0]
def inner_angle(v,w):
   cosx=dot_product(v,w)/(length(v)*length(w))
   rad=acos(cosx) # in radians
   return rad*180/pi # returns degrees
def angle_clockwise(A, B):
    inner=inner_angle(A,B)
    det = determinant(A,B)
    print "det:"
    print det 
    if det<0: #this is a property of the det. If the det < 0 then B is clockwise of A
        return inner
    else: # if the det > 0 then A is immediately clockwise of B
        return 360-inner
#------------------------------------------

def callback(data):
    # Setup the tf broadcaster, 
    # then just use the pose values from the ground truth pose
    br = tf.TransformBroadcaster()
    br.sendTransform((data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z),
                     (data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w),
                     rospy.Time.now(),
                     "base_link",
                     "odom")
    # Next publish the odometry
    odom = Odometry()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = data.pose.pose.position.x
    odom.pose.pose.position.y = data.pose.pose.position.y
    odom.pose.pose.orientation.x = data.pose.pose.orientation.x
    odom.pose.pose.orientation.y = data.pose.pose.orientation.y
    odom.pose.pose.orientation.z = data.pose.pose.orientation.z
    odom.pose.pose.orientation.w = data.pose.pose.orientation.w
    # Calc the velocity
    # linear.x in base_link frame is made up of both linear.x & .y in the odom frame
    odom.twist.twist.linear.x = sqrt(pow(data.twist.twist.linear.x,2)+pow(data.twist.twist.linear.y,2))
    # but the sign is harder to calc

    global odom_prev
    # To find the direction (fwd / rev), we must calculate the angle
    # between current position and the previous position.
    A = [odom.pose.pose.position.x, odom.pose.pose.position.y]
    B = [odom_prev.pose.pose.position.x, odom_prev.pose.pose.position.y]

    alpha = atan2(A[1]-B[1], A[0]-B[0])*180/pi
    # print "alpha: {:.2f}".format(alpha)
    
    # Then, we get the actual heading (yaw angle)
    _,_,theta = euler_from_quaternion([odom.pose.pose.orientation.x,
                              odom.pose.pose.orientation.y,
                              odom.pose.pose.orientation.z,
                              odom.pose.pose.orientation.w])
    theta = theta*180/pi # convert to deg

    # Comparing these two values will give us the car's driving direction
    # If its driving forward, the two angles should be about the same
    # If its driving in reverse, the value will differ greatly
    if abs(alpha-theta)>45:
        odom.twist.twist.linear.x = -odom.twist.twist.linear.x
    # print "theta: {:.2f}\n---".format(theta)
    
    # whereas the angular.z twist is just the same
    odom.twist.twist.angular.z = data.twist.twist.angular.z
    global pub_odom
    pub_odom.publish(odom)
    odom_prev = odom


def listener():
    rospy.init_node('odom_pub', anonymous=True)

    # So we subscribe to this ground truth pose, then use it to publish a tf between 
    # base_link and odom
    rospy.Subscriber("/base_pose_ground_truth", Odometry, callback)
    global pub_odom
    pub_odom = rospy.Publisher('/odom', Odometry, queue_size=1)
    
    global odom_prev
    odom_prev = Odometry()

    rospy.spin()

if __name__ == '__main__':
    listener()
