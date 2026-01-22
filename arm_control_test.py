#!/usr/bin/env python3
import rospy
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class ArmController:
    def __init__(self):
        rospy.init_node('arm_controller_test', anonymous=True)
        
        self.joint_publishers = {}
        joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint']
        for joint in joints:
            topic_name = f'/{joint}_controller/command'
            self.joint_publishers[joint] = rospy.Publisher(topic_name, Float64, queue_size=10)
        
        self.joint_states_sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        
        time.sleep(1)
        rospy.loginfo("Arm controller initialized")

    def joint_states_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.joint_publishers:
                rospy.loginfo(f"Current {name} position: {msg.position[i]}")

    def move_joint(self, joint_name, position):
        if joint_name in self.joint_publishers:
            msg = Float64()
            msg.data = position
            self.joint_publishers[joint_name].publish(msg)
            rospy.loginfo(f"Moving {joint_name} to position {position}")
        else:
            rospy.logwarn(f"Joint {joint_name} not found")

    def test_movement(self):
        test_positions = {
            'joint1': 0.5,
            'joint2': 0.3,
            'joint3': 0.2,
            'joint4': 0.1,
            'joint5': 0.0,
            'r_joint': 0.0
        }
        
        for joint, pos in test_positions.items():
            self.move_joint(joint, pos)
            time.sleep(2)

if __name__ == '__main__':
    try:
        controller = ArmController()
        rospy.loginfo("Starting test movement...")
        controller.test_movement()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 