#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

def callback(data):
    rospy.loginfo("message: %s", data.data)

def listener():
    rospy.init_node('arm_command_listener', anonymous=True)
    rospy.Subscriber('/arm_controller/command', Float64MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
