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


def callback(data):
    # Setup the tf broadcaster, 
    # then just use the pose values from the ground truth pose
    br = tf.TransformBroadcaster()
    br.sendTransform((data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z),
                     (data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w),
                     rospy.Time.now(),
                     "base_link",
                     "odom")

def listener():
    rospy.init_node('listener', anonymous=True)

    # So we subscribe to this ground truth pose, then use it to publish a tf between 
    # base_link and odom
    rospy.Subscriber("/base_pose_ground_truth", Odometry, callback)
    
    rospy.spin()

if __name__ == '__main__':
    listener()
