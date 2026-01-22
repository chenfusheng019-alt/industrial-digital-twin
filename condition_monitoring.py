#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
## Simple talker demo that published std_msgs/Strings messages
## to the 'condition-monitoring' topic

import rospy
import random
from cm.msg import msg_cm as RosJointState
from sensor_msgs.msg import JointState


class CMDataPublisher:
    def __init__(self, freq=2):
        rospy.init_node('condition_monitoring_data_publisher', anonymous=True)
        rate = rospy.get_param('~rate', freq)
        r = rospy.Rate(rate)

        self.msg = RosJointState()
        self.msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint']
        self.msg.header.frame_id = 'not_relervant'
        self.msg.position = [0]*6
        self.msg.temperature = [0]*6
        self.msg.voltage = [0]*6

        # Subscribe to joint states
        self.joint_states_sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        
        # Start publisher
        self.joint_states_pub = rospy.Publisher('/condition_monitoring', RosJointState, queue_size=50)

        rospy.loginfo("Starting Joint State Publisher at " + str(rate) + "Hz")
        rospy.loginfo("Waiting for joint states...")

        while not rospy.is_shutdown():
            self.condition_monitoring()
            r.sleep()

    def joint_states_callback(self, msg):
        # Update positions from joint states
        for i, name in enumerate(self.msg.name):
            if name in msg.name:
                idx = msg.name.index(name)
                old_pos = self.msg.position[i]
                self.msg.position[i] = msg.position[idx]
                # Print debug info if position changed
                if old_pos != self.msg.position[i]:
                    rospy.loginfo(f"Joint {name} position changed: {old_pos} -> {self.msg.position[i]}")

    def condition_monitoring(self):
        # Log current time.
        self.msg.header.stamp = rospy.Time.now()
        # Generate random data for temperature and voltage
        for motor_idx in range(6):
            # Generate random temperature between 20 and 50 degrees
            self.msg.temperature[motor_idx] = random.uniform(20, 50)
            # Generate random voltage between 4.5 and 5.5 volts
            self.msg.voltage[motor_idx] = random.uniform(4.5, 5.5)
        # Publish the data.
        self.joint_states_pub.publish(self.msg)
        # Print current positions
        rospy.loginfo(f"Published positions: {self.msg.position}")


if __name__ == '__main__':
    try:
        s = CMDataPublisher(2)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

