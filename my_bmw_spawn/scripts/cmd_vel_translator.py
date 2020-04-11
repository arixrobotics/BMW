#!/usr/bin/env python

# Reads the topic /cmd_vel of type geometry_msgs.Twist
# Then converts it appropriately to prius_msgs.Control message 
# and publish to /prius to control the car


import rospy
from prius_msgs.msg import Control
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
# from math import abs 

class Translator:
    def __init__(self):
        self.sub = rospy.Subscriber("/cmd_vel", Twist, self.callback)
        self.pub = rospy.Publisher('/prius', Control, queue_size=1)
        # self.last_published_time = rospy.get_rostime()
        # self.last_published = None
        # self.timer = rospy.Timer(rospy.Duration(1./20.), self.timer_callback)
        # rospy.logwarn("key_translater READY...")

    # def timer_callback(self, event):
    #     if self.last_published and self.last_published_time < rospy.get_rostime() + rospy.Duration(1.0/20.):
    #         self.callback(self.last_published)

    def callback(self, message):
        throttle_value = message.linear.x
        # gears_value = message.linear.z
        steering_value = message.angular.z * 10.0

        # rospy.logdebug("joy_translater received Throttle value "+str(throttle_value))
        # rospy.logdebug("joy_translater received Gears value "+str(gears_value))
        # rospy.logdebug("joy_translater received Steering value "+str(steering_value))
        command = Control()

        header = Header()
        header.frame_id = "world"
        header.stamp = rospy.Time.now()

        command.header = header
        if throttle_value > 0.0:
            command.shift_gears = Control.FORWARD
            command.throttle = throttle_value
            command.brake = 0.0
        elif throttle_value < 0.0:
            command.shift_gears = Control.REVERSE
            command.throttle = abs(throttle_value)
            command.brake = 0.0
            steering_value = -steering_value
        else:
            command.shift_gears = Control.NEUTRAL
            command.throttle = 0.0
            command.brake = 1.0

        # if gears_value > 0.0:
        #     command.shift_gears = Control.FORWARD
        # elif gears_value == 0.0:
        #     command.shift_gears = Control.NEUTRAL
        # elif gears_value < 0.0:
        #     command.shift_gears = Control.REVERSE
        # else:
        #     command.shift_gears = Control.NO_COMMAND

        command.steer = steering_value
        # self.last_published = message
        self.pub.publish(command)
        print command

if __name__ == '__main__':
    rospy.init_node('cmd_vel_translator')#, log_level=rospy.DEBUG)
    t = Translator()
    rospy.spin()
