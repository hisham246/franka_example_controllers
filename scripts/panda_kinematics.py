#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import numpy as np
import roboticstoolbox as rtb

class PandaKinematics:
    def __init__(self):
        rospy.init_node('panda_kinematics', anonymous=True)
        self.joint_angles = None
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        # Define DH parameters for Panda robot
        self.panda = rtb.models.DH.Panda()

    def joint_state_callback(self, msg):
        # Extract the first 7 elements from the position array
        self.joint_angles = msg.position[:7]

    def calculate_fkine(self):
        if self.joint_angles:
            # Convert joint angles to a numpy array
            q = list(self.joint_angles)

            # Calculate forward kinematics using Robotics Toolbox
            T = self.panda.fkine(q)

            rospy.loginfo("Transformation Matrix T:\n%s", T)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.calculate_fkine()
            rate.sleep()

if __name__ == '__main__':
    try:
        kinematics = PandaKinematics()
        kinematics.run()
    except rospy.ROSInterruptException:
        pass
