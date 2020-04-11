#!/usr/bin/env python

# Reads the topic /cmd_vel of type geometry_msgs.Twist
# and also the odometry of the vehicle,
# Then converts it appropriately to prius_msgs.Control message 
# and publish to /prius to control the car


import rospy
from prius_msgs.msg import Control
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from nav_msgs.msg import Odometry

# These are the target and current speeds
global target
global current
target = Twist()
current = Twist()

def callback_cmd(message):
    global target
    target.linear.x = message.linear.x 
    target.angular.z = message.angular.z

def callback_odom(message):
    global current
    current.linear.x = message.twist.twist.linear.x
    current.angular.z = message.twist.twist.angular.z 


def translator():
    global target
    global current 

    rospy.init_node("translator", anonymous=False)
    sub_cmd = rospy.Subscriber("/cmd_vel", Twist, callback_cmd)
    sub_odom = rospy.Subscriber("/odom", Odometry, callback_odom)
    pub_prius = rospy.Publisher('/prius', Control, queue_size=1)

    kplx = 2.0 #0.5  # P-constant for linear x control
    kpaz = 2.0  # P-constant for angular z control
    kdlx = 50
    kdaz = 0

    r = rospy.Rate(20)

    command = Control()
    command_prev = Control()

    lx_err = 0
    az_err = 0
    az_err_p = 0

    while not rospy.is_shutdown():
        # Only move if the command is above certain threshold
        th = 0.01
        if (target.linear.x >= th) or (target.linear.x <= -th):

            # calc the error to linear speed
            lx_err = target.linear.x - current.linear.x
            command.throttle = kplx * lx_err
            command.brake = 0.0

            # get the driving direction
            if target.linear.x > 0.0:
                command.shift_gears = Control.FORWARD
            else:
                command.shift_gears = Control.REVERSE
                command.throttle = -command.throttle
            # check for braking
            if command.throttle < 0.0:  
                command.brake = -command.throttle
                command.throttle = 0.0

            # Next the steering
            # calc the error to angular speed
            target.angular.z = 1.2*target.angular.z
            az_err = target.angular.z - current.angular.z
            command.steer = kpaz * az_err - kdaz * (az_err - az_err_p)
            # print "az_err: {}".format(az_err)
            # print "az_erp: {}".format(az_err_p)
            # print "az_dif: {}".format(az_err - az_err_p)

            # check for reverse
            if command.shift_gears == Control.REVERSE:
                command.steer = -command.steer

            # check for upper bounds
            if command.throttle > 1.0:
                command.throttle = 1.0
            if command.steer > 1.0:
                command.steer = 1.0
            if command.steer < -1.0:
                command.steer = -1.0

            az_err_p = az_err

            print "Linear control:"
            print "target : {:.2f}".format(target.linear.x)
            print "current: {:.2f}".format(current.linear.x)
            print "err    : {:.2f}".format(lx_err)
            print "cmd_str: {:.2f}".format(command.throttle)
            print "gear: {}".format(command.shift_gears)
            print "\nAngular control:"
            print "target : {:.2f}".format(target.angular.z)
            print "current: {:.2f}".format(current.angular.z)
            print "err    : {:.2f}".format(az_err)
            print "cmd_str: {:.2f}".format(command.steer)
            print "gear: {}".format(command.shift_gears)
        else:
            command.shift_gears = Control.NEUTRAL
            command.throttle = 0.0
            command.steer = 0.0
            command.brake = 1.0
        
        pub_prius.publish(command)
        print "---"
        # print lx_err
        # print command
        r.sleep()


if __name__ == '__main__':
    translator()