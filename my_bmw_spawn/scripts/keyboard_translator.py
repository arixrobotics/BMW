#!/usr/bin/env python

# Copyright 2017 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rospy
from prius_msgs.msg import Control
from geometry_msgs.msg import Twist
from std_msgs.msg import Header

class Translator:
    def __init__(self):
        self.sub = rospy.Subscriber("/cmd_vel", Twist, self.callback)
        self.pub = rospy.Publisher('prius', Control, queue_size=1)
        self.last_published_time = rospy.get_rostime()
        self.last_published = None
        self.timer = rospy.Timer(rospy.Duration(1./20.), self.timer_callback)
        rospy.logwarn("key_translater READY...")

    def timer_callback(self, event):
        if self.last_published and self.last_published_time < rospy.get_rostime() + rospy.Duration(1.0/20.):
            self.callback(self.last_published)

    def callback(self, message):
        throttle_value = message.linear.x
        gears_value = message.linear.z
        steering_value = message.angular.z

        rospy.logdebug("joy_translater received Throttle value "+str(throttle_value))
        rospy.logdebug("joy_translater received Gears value "+str(gears_value))
        rospy.logdebug("joy_translater received Steering value "+str(steering_value))
        command = Control()

        header = Header()
        header.frame_id = "world"
        header.stamp = rospy.Time.now()

        command.header = header
        if throttle_value >= 0:
            command.throttle = throttle_value
            command.brake = 0.0
        else:
            command.brake = throttle_value * -1
            command.throttle = 0.0

        if gears_value > 0.0:
            command.shift_gears = Control.FORWARD
        elif gears_value == 0.0:
            command.shift_gears = Control.NEUTRAL
        elif gears_value < 0.0:
            command.shift_gears = Control.REVERSE
        else:
            command.shift_gears = Control.NO_COMMAND

        command.steer = steering_value
        self.last_published = message
        self.pub.publish(command)

if __name__ == '__main__':
    rospy.init_node('keyboard_translator', log_level=rospy.DEBUG)
    t = Translator()
    rospy.spin()
